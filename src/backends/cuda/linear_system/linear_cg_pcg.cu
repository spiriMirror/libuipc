#include <linear_system/linear_cg_pcg.h>
#include <sim_engine.h>
#include <linear_system/global_linear_system.h>
#include <uipc/common/timer.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LinearCGPCG);

void LinearCGPCG::do_build(BuildInfo& info)
{
    auto& config = world().scene().config();

    auto solver_attr = config.find<std::string>("linear_system/solver");
    std::string solver_name = solver_attr ? solver_attr->view()[0] : std::string{"pcg"};
    if(solver_name != "cg_pcg")
    {
        throw SimSystemException("LinearCGPCG unused");
    }

    auto& global_linear_system = require<GlobalLinearSystem>();

    max_iter_ratio = 2;

    auto tol_rate_attr = config.find<Float>("linear_system/tol_rate");
    global_tol_rate    = tol_rate_attr->view()[0];

    logger::info("LinearCGPCG: max_iter_ratio = {}, tol_rate = {}",
                 max_iter_ratio,
                 global_tol_rate);
}

void LinearCGPCG::do_solve(GlobalLinearSystem::SolvingInfo& info)
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
    }

    r.resize(N);
    z.resize(N);
    p.resize(N);
    Ap.resize(N);

    auto iter = cg_pcg(x, b, max_iter_ratio * b.size());

    info.iter_count(iter);
}

// Fused update: x += alpha*p, r -= alpha*Ap, p = z + beta*p
// p is both read (for x update) and written, handled correctly within one thread.
static void cg_pcg_update(Float                        alpha,
                           Float                        beta,
                           muda::DenseVectorView<Float> x,
                           muda::DenseVectorView<Float> r,
                           muda::CDenseVectorView<Float> Ap,
                           muda::DenseVectorView<Float> p,
                           muda::CDenseVectorView<Float> z)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(r.size(),
               [alpha = alpha,
                beta  = beta,
                x     = x.viewer().name("x"),
                r     = r.viewer().name("r"),
                Ap    = Ap.cviewer().name("Ap"),
                p     = p.viewer().name("p"),
                z     = z.cviewer().name("z")] __device__(int i) mutable
               {
                   Float pi = p(i);
                   x(i) += alpha * pi;
                   r(i) -= alpha * Ap(i);
                   p(i) = z(i) + beta * pi;
               });
}

SizeT LinearCGPCG::cg_pcg(muda::DenseVectorView<Float> x,
                            muda::CDenseVectorView<Float> b,
                            SizeT                         max_iter)
{
    Timer cg_pcg_timer{"CG_PCG"};

    SizeT k = 0;

    // r = b  (x0 = 0, so r = b - A*0 = b)
    r.buffer_view().copy_from(b.buffer_view());

    // z = P^{-1} * r
    {
        Timer timer{"Apply Preconditioner"};
        apply_preconditioner(z, r);
    }

    // p = z
    p = z;

    // rz = r^T * z
    Float rz = ctx().dot(r.cview(), z.cview());
    Float abs_rz0 = std::abs(rz);

    // check convergence
    if(accuracy_statisfied(r) && abs_rz0 == Float{0.0})
        return 0;

    for(k = 1; k < max_iter; ++k)
    {
        // Ap = A * p
        {
            Timer timer{"SpMV"};
            spmv(p.cview(), Ap.view());
        }

        // Chronopoulos-Gear: apply preconditioner BEFORE the dot products
        // z = P^{-1} * r  (uses current r, before the x/r update)
        {
            Timer timer{"Apply Preconditioner"};
            apply_preconditioner(z, r);
        }

        // Issue both dot products consecutively (two blocking reduces, but
        // preconditioner is already done so no extra work is hidden between them)
        Float pAp    = ctx().dot(p.cview(), Ap.cview());
        Float rz_new = ctx().dot(r.cview(), z.cview());

        Float alpha = rz / pAp;
        Float beta  = rz_new / rz;

        // check convergence (before applying updates so r is still valid)
        if(accuracy_statisfied(r) && std::abs(rz_new) <= global_tol_rate * abs_rz0)
            break;

        // Fused update: x += alpha*p, r -= alpha*Ap, p = z + beta*p
        cg_pcg_update(alpha, beta, x, r.view(), Ap.cview(), p.view(), z.cview());

        rz = rz_new;
    }

    return k;
}
}  // namespace uipc::backend::cuda
