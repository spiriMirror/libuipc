#include <linear_system/linear_pcg.h>
#include <sim_engine.h>
#include <linear_system/global_linear_system.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LinearPCG);

void LinearPCG::do_build(BuildInfo& info)
{
    auto& global_linear_system = require<GlobalLinearSystem>();

    // TODO: get info from the scene, now we just use the default value
    max_iter_ratio  = 2;
    global_tol_rate = world().scene().info()["linear_system"]["tol_rate"];
    // spdlog::info("LinearPCG: max_iter_ratio = {}, tol_rate = {}", max_iter_ratio, global_tol_rate);
}

void LinearPCG::do_solve(GlobalLinearSystem::SolvingInfo& info)
{
    auto x = info.x();
    auto b = info.b();

    x.buffer_view().fill(0);

    auto N = x.size();
    if(z.capacity() < N)
    {
        auto M = reserve_ratio * N;
        z.reserve(M);
        p.reserve(M);
        r.reserve(M);
        Ap.reserve(M);
    }

    z.resize(N);
    p.resize(N);
    r.resize(N);
    Ap.resize(N);

    auto iter = pcg(x, b, max_iter_ratio * b.size());

    info.iter_count(iter);
}

SizeT LinearPCG::pcg(muda::DenseVectorView<Float> x, muda::CDenseVectorView<Float> b, SizeT max_iter)
{
    SizeT k = 0;
    // r = b - A * x
    {
        // r = b;
        r.buffer_view().copy_from(b.buffer_view());

        // x == 0, so we don't need to do the following
        // r = - A * x + r
        //spmv(-1.0, x.as_const(), 1.0, r.view());
    }

    Float alpha, beta, rz, rz0;

    // z = P * r (apply preconditioner)
    apply_preconditioner(z, r);

    // p = z
    p = z;

    // init rz
    // rz = r^T * z
    rz = ctx().dot(r.cview(), z.cview());

    rz0 = std::abs(rz);

    if constexpr(RUNTIME_CHECK)
    {
        if(std::isnan(rz0) || !std::isfinite(rz0))
        {
            auto norm_r = ctx().norm(r.cview());
            auto norm_z = ctx().norm(z.cview());

            UIPC_ASSERT(!std::isnan(rz0) && std::isfinite(rz0),
                        "Init Residual is {}, norm(r) = {}, norm(z) = {}",
                        rz0,
                        norm_r,
                        norm_z);
        }
    }

    // check convergence
    if(accuracy_statisfied(r) && rz0 == Float{0.0})
        return 0;

    for(k = 1; k < max_iter; ++k)
    {
        spmv(p.cview(), Ap.view());

        // alpha = rz / dot(p.cview(), Ap.cview());
        alpha = rz / ctx().dot(p.cview(), Ap.cview());

        // x = x + alpha * p
        ctx().axpby(alpha, p.cview(), Float{1}, x);

        // r = r - alpha * Ap
        ctx().axpby(-alpha, Ap.cview(), Float{1}, r.view());

        // z = P * r (apply preconditioner)
        apply_preconditioner(z, r);

        // rz_new = r^T * z
        // rz_new = dot(r.cview(), z.cview());
        Float rz_new = ctx().dot(r.cview(), z.cview());

        if constexpr(RUNTIME_CHECK)
        {
            if(std::isnan(rz_new) || !std::isfinite(rz_new))
            {
                auto norm_r = ctx().norm(r.cview());
                auto norm_z = ctx().norm(z.cview());
                UIPC_ASSERT(!std::isnan(rz_new) && std::isfinite(rz_new),
                            "Residual is {}, norm(r) = {}, norm(z) = {}",
                            rz_new,
                            norm_r,
                            norm_z);
            }
        }

        // check convergence
        if(accuracy_statisfied(r) && std::abs(rz_new) <= global_tol_rate * rz0)
            break;

        // beta = rz_new / rz
        beta = rz_new / rz;

        // p = z + beta * p
        ctx().axpby(Float{1}, z.cview(), beta, p.view());

        // update rz
        rz = rz_new;
    }

    return k;
}
}  // namespace uipc::backend::cuda
