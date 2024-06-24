#include <linear_system/linear_pcg.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<LinearPCG>
{
  public:
    static U<LinearPCG> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();
        if(info["linear_system"]["solver"] == "linear_pcg")
            return make_unique<LinearPCG>(engine);
        else
            return nullptr;
    }
};

REGISTER_SIM_SYSTEM(LinearPCG);

void LinearPCG::do_build()
{
    on_init_scene(
        [this]
        {
            // TODO: get info from the scene, now we just use the default value
            max_iter_ratio  = 2.0;
            global_tol_rate = 1e-4;
            spdlog::info("LinearPCG: max_iter_ratio = {}, global_tol_rate = {}",
                         max_iter_ratio,
                         global_tol_rate);
        });
}

void LinearPCG::do_solve(GlobalLinearSystem::SolvingInfo& info)
{
    auto x = info.x();
    auto b = info.b();

    x.buffer_view().fill(0);

    auto N = x.size();
    if(z.size() < N)
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

    // check convergence
    if(accuracy_statisfied(r) && std::abs(rz) <= global_tol_rate * rz0)
        return k;

    for(k = 1; k < max_iter; ++k)
    {
        spmv(p.cview(), Ap.view());

        // alpha = rz / dot(p.cview(), Ap.cview());
        alpha = rz / ctx().dot(p.cview(), Ap.cview());

        // x = x + alpha * p
        ctx().axpby(alpha, p.cview(), 1.0, x);

        // r = r - alpha * Ap
        ctx().axpby(-alpha, Ap.cview(), 1.0, r.view());


        // check convergence
        if(accuracy_statisfied(r) && std::abs(rz) <= global_tol_rate * rz0)
            break;

        // z = P * r (apply preconditioner)
        apply_preconditioner(z, r);

        // rz_new = r^T * z
        // rz_new = dot(r.cview(), z.cview());
        Float rz_new = ctx().dot(r.cview(), z.cview());

        // beta = rz_new / rz
        beta = rz_new / rz;

        // p = z + beta * p
        ctx().axpby(1.0, z.cview(), beta, p.view());

        // update rz
        rz = rz_new;
    }

    return k;
}
}  // namespace uipc::backend::cuda
