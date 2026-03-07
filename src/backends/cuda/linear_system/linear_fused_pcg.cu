#include <linear_system/linear_fused_pcg.h>
#include <sim_engine.h>
#include <linear_system/global_linear_system.h>
#include <uipc/common/timer.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LinearFusedPCG);

void LinearFusedPCG::do_build(BuildInfo& info)
{
    auto& config = world().scene().config();

    auto solver_attr = config.find<std::string>("linear_system/solver");
    std::string solver_name = solver_attr ? solver_attr->view()[0] : std::string{"fused_pcg"};
    if(solver_name != "fused_pcg")
    {
        throw SimSystemException("LinearFusedPCG unused");
    }

    auto& global_linear_system = require<GlobalLinearSystem>();

    max_iter_ratio = 2;

    auto tol_rate_attr = config.find<Float>("linear_system/tol_rate");
    global_tol_rate    = tol_rate_attr->view()[0];

    auto check_attr = config.find<IndexT>("linear_system/check_interval");
    if(check_attr)
        check_interval = check_attr->view()[0];

    logger::info("LinearFusedPCG: max_iter_ratio = {}, tol_rate = {}, check_interval = {}",
                 max_iter_ratio,
                 global_tol_rate,
                 check_interval);
}

void LinearFusedPCG::do_solve(GlobalLinearSystem::SolvingInfo& info)
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

    auto iter = fused_pcg(x, b, max_iter_ratio * b.size());

    info.iter_count(iter);
}

// d_result = x^T * y  (cublas-free, device-only)
void fused_dot(muda::CDenseVectorView<Float> x,
               muda::CDenseVectorView<Float> y,
               muda::VarView<Float>          d_result)
{
    using namespace muda;

    cudaMemsetAsync(d_result.data(), 0, sizeof(Float));

    constexpr int block_dim = 256;
    constexpr int warp_size = 32;
    int           n         = x.size();
    int           block_count = (n + block_dim - 1) / block_dim;

    Launch(block_count, block_dim)
        .file_line(__FILE__, __LINE__)
        .apply(
               [x        = x.cviewer().name("x"),
                y        = y.cviewer().name("y"),
                d_result = d_result.data(),
                n] __device__() mutable
               {
                   int i = blockIdx.x * blockDim.x + threadIdx.x;
                   Float val = (i < n) ? x(i) * y(i) : Float(0);

                   for(int offset = warp_size / 2; offset > 0; offset /= 2)
                       val += __shfl_down_sync(0xFFFFFFFF, val, offset);

                   if((threadIdx.x & (warp_size - 1)) == 0)
                       atomicAdd(d_result, val);
               });
}

// x += alpha*p, r -= alpha*Ap  (alpha = d_rz/d_pAp on device, skips if converged)
void fused_update_xr(muda::CVarView<Float>         d_rz,
                     muda::CVarView<Float>         d_pAp,
                     muda::DenseVectorView<Float>  x,
                     muda::CDenseVectorView<Float> p,
                     muda::DenseVectorView<Float>  r,
                     muda::CDenseVectorView<Float> Ap,
                     muda::CVarView<int>           d_converged)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(r.size(),
               [d_rz        = d_rz.cviewer().name("d_rz"),
                d_pAp       = d_pAp.cviewer().name("d_pAp"),
                x           = x.viewer().name("x"),
                p           = p.cviewer().name("p"),
                r           = r.viewer().name("r"),
                Ap          = Ap.cviewer().name("Ap"),
                d_converged = d_converged.cviewer().name("d_converged")] __device__(int i) mutable
               {
                   if(*d_converged) return;
                   Float alpha = *d_rz / *d_pAp;
                   x(i) += alpha * p(i);
                   r(i) -= alpha * Ap(i);
               });
}

// p = z + beta*p (beta = rz_new/rz on device), convergence detection, rz swap
void fused_update_p(muda::VarView<Float>          d_rz_new,
                    muda::VarView<Float>          d_rz,
                    muda::DenseVectorView<Float>  p,
                    muda::CDenseVectorView<Float> z,
                    Float                         rz_tol,
                    muda::VarView<int>            d_converged)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(p.size(),
               [d_rz_new    = d_rz_new.viewer().name("d_rz_new"),
                d_rz        = d_rz.viewer().name("d_rz"),
                p           = p.viewer().name("p"),
                z           = z.cviewer().name("z"),
                rz_tol,
                d_converged = d_converged.viewer().name("d_converged")] __device__(int i) mutable
               {
                   if(*d_converged) return;
                   Float rz_old = *d_rz;
                   Float rz_new = *d_rz_new;
                   if(abs(rz_new) <= rz_tol)
                   {
                       if(i == 0) *d_converged = 1;
                       return;
                   }
                   Float beta = rz_new / rz_old;
                   p(i) = z(i) + beta * p(i);
                   if(i == 0) *d_rz = rz_new;
               });
}

SizeT LinearFusedPCG::fused_pcg(muda::DenseVectorView<Float>  x,
                                muda::CDenseVectorView<Float> b,
                                SizeT                         max_iter)
{
    Timer pcg_timer{"FusedPCG"};

    SizeT k = 0;

    // r = b - A*x, but x0 = 0 so r = b
    r.buffer_view().copy_from(b.buffer_view());

    // z = P^{-1} * r
    {
        Timer timer{"Apply Preconditioner"};
        apply_preconditioner(z, r);
    }

    // p = z
    p = z;

    // rz = r^T * z
    fused_dot(r.cview(), z.cview(), d_rz.view());
    Float rz_host = d_rz;
    Float abs_rz0 = std::abs(rz_host);

    if(accuracy_statisfied(r) && abs_rz0 == Float{0.0})
        return 0;

    Float rz_tol = global_tol_rate * abs_rz0;
    d_converged = 0;

    for(k = 1; k < max_iter; ++k)
    {
        // Ap = A * p,  pAp = p^T * Ap
        {
            Timer timer{"SpMV"};
            spmv_dot(p.cview(), Ap.view(), d_pAp.view());
        }

        // alpha = rz / pAp,  x += alpha * p,  r -= alpha * Ap
        fused_update_xr(d_rz.view(), d_pAp.view(), x, p.cview(), r.view(), Ap.cview(), d_converged.view());

        // z = P^{-1} * r
        {
            Timer timer{"Apply Preconditioner"};
            apply_preconditioner(z, r);
        }

        // rz_new = r^T * z
        fused_dot(r.cview(), z.cview(), d_rz_new.view());

        if(k % check_interval == 0)
        {
            if(accuracy_statisfied(r))
            {
                int converged_host = d_converged;
                if(converged_host)
                    break;
                Float rz_new_host = d_rz_new;
                if(!std::isfinite(rz_new_host)) [[unlikely]]
                {
                    auto norm_r = ctx().norm(r.cview());
                    auto norm_z = ctx().norm(z.cview());
                    UIPC_ASSERT(false,
                                "Frame {}, Newton {}, FusedPCG Iter {}: r^T*z = {}, "
                                "norm(r) = {}, norm(z) = {}.",
                                engine().frame(),
                                engine().newton_iter(),
                                k,
                                rz_new_host,
                                norm_r,
                                norm_z);
                }
            }
        }

        // beta = rz_new / rz,  p = z + beta * p,  rz = rz_new
        fused_update_p(d_rz_new.view(), d_rz.view(), p.view(), z.cview(), rz_tol, d_converged.view());
    }

    return k;
}
}  // namespace uipc::backend::cuda
