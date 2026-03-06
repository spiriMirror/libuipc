#include <linear_system/linear_pipe_pcg.h>
#include <sim_engine.h>
#include <linear_system/global_linear_system.h>
#include <uipc/common/timer.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LinearPipePCG);

void LinearPipePCG::do_build(BuildInfo& info)
{
    auto& config = world().scene().config();

    auto solver_attr = config.find<std::string>("linear_system/solver");
    std::string solver_name = solver_attr ? solver_attr->view()[0] : std::string{"pcg"};
    if(solver_name != "pipe_pcg")
    {
        throw SimSystemException("LinearPipePCG unused");
    }

    auto& global_linear_system = require<GlobalLinearSystem>();

    max_iter_ratio = 2;

    auto tol_rate_attr = config.find<Float>("linear_system/tol_rate");
    global_tol_rate    = tol_rate_attr->view()[0];

    logger::info("LinearPipePCG: max_iter_ratio = {}, tol_rate = {}",
                 max_iter_ratio,
                 global_tol_rate);
}

void LinearPipePCG::do_solve(GlobalLinearSystem::SolvingInfo& info)
{
    auto x = info.x();
    auto b = info.b();

    x.buffer_view().fill(0);

    auto N = x.size();
    if(r.capacity() < N)
    {
        auto M = reserve_ratio * N;
        r.reserve(M);
        z.reserve(M);
        p.reserve(M);
        Ap.reserve(M);
        w.reserve(M);
    }

    r.resize(N);
    z.resize(N);
    p.resize(N);
    Ap.resize(N);
    w.resize(N);

    auto iter = pipe_pcg(x, b, max_iter_ratio * b.size());

    info.iter_count(iter);
}

SizeT LinearPipePCG::pipe_pcg(muda::DenseVectorView<Float> x,
                               muda::CDenseVectorView<Float> b,
                               SizeT                          max_iter)
{
    using namespace muda;
    Timer pipe_pcg_timer{"PipePCG"};

    SizeT k = 0;

    // Initialization (x0 = 0):
    // r = b
    r.buffer_view().copy_from(b.buffer_view());

    // z = P^{-1} * r
    {
        Timer timer{"Apply Preconditioner"};
        apply_preconditioner(z, r);
    }

    // p = z
    p = z;

    // At initialization p == z, so Ap = A*p = A*z.
    // Compute Ap once; copy result to w (both are equal here).
    {
        Timer timer{"SpMV"};
        spmv(p.cview(), Ap.view());
    }
    w = Ap;

    // rz = dot(r, z)
    Float rz      = ctx().dot(r.cview(), z.cview());
    Float abs_rz0 = std::abs(rz);

    // check convergence
    if(accuracy_statisfied(r) && abs_rz0 == Float{0.0})
        return 0;

    for(k = 1; k < max_iter; ++k)
    {
        // pAp = dot(p, Ap)  [blocking reduce #1]
        Float pAp   = ctx().dot(p.cview(), Ap.cview());
        Float alpha = rz / pAp;

        // x += alpha * p
        // r -= alpha * Ap
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(r.size(),
                   [alpha = alpha,
                    x     = x.viewer().name("x"),
                    r     = r.viewer().name("r"),
                    p     = p.cview().cviewer().name("p"),
                    Ap    = Ap.cview().cviewer().name("Ap")] __device__(int i) mutable
                   {
                       x(i) += alpha * p(i);
                       r(i) -= alpha * Ap(i);
                   });

        // z = P^{-1} * r_new  (pipelined: preconditioning before the dot)
        {
            Timer timer{"Apply Preconditioner"};
            apply_preconditioner(z, r);
        }

        // w = A * z  (look-ahead SpMV: issued before blocking on dot)
        {
            Timer timer{"SpMV"};
            spmv(z.cview(), w.view());
        }

        // rz_new = dot(r, z)  [blocking reduce #2]
        Float rz_new = ctx().dot(r.cview(), z.cview());

        // check convergence
        if(accuracy_statisfied(r) && std::abs(rz_new) <= global_tol_rate * abs_rz0)
            break;

        Float beta = rz_new / rz;

        // p  = z + beta * p
        // Ap = w + beta * Ap   (recurrence: avoids a standalone SpMV next iteration)
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(r.size(),
                   [beta = beta,
                    p    = p.viewer().name("p"),
                    z    = z.cview().cviewer().name("z"),
                    Ap   = Ap.viewer().name("Ap"),
                    w    = w.cview().cviewer().name("w")] __device__(int i) mutable
                   {
                       Float pi  = p(i);
                       Float api = Ap(i);
                       p(i)  = z(i) + beta * pi;
                       Ap(i) = w(i) + beta * api;
                   });

        rz = rz_new;
    }

    return k;
}
}  // namespace uipc::backend::cuda

