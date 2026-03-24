#include <linear_system/linear_pcg.h>
#include <sim_engine.h>
#include <linear_system/global_linear_system.h>
#include <cuda_device/builtin.h>
#include <utils/matrix_market.h>
#include <backends/common/backend_path_tool.h>
#include <uipc/common/timer.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LinearPCG);

void LinearPCG::do_build(BuildInfo& info)
{
    auto& config = world().scene().config();

    auto solver_attr = config.find<std::string>("linear_system/solver");
    UIPC_ASSERT(solver_attr, "linear_system/solver not found");
    if(solver_attr->view()[0] != "linear_pcg")
    {
        throw SimSystemException("LinearPCG unused");
    }

    auto& global_linear_system = require<GlobalLinearSystem>();

    // TODO: get info from the scene, now we just use the default value
    max_iter_ratio = 2;

    auto tol_rate_attr = config.find<Float>("linear_system/tol_rate");
    UIPC_ASSERT(tol_rate_attr, "linear_system/tol_rate not found");
    global_tol_rate = tol_rate_attr->view()[0];

    auto dump_attr = config.find<IndexT>("extras/debug/dump_linear_pcg");
    UIPC_ASSERT(dump_attr, "extras/debug/dump_linear_pcg not found");
    need_debug_dump = dump_attr->view()[0];

    logger::info("LinearPCG: max_iter_ratio = {}, tol_rate = {}, debug_dump = {}",
                 max_iter_ratio,
                 global_tol_rate,
                 need_debug_dump);
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
    d_converged_false = 0;

    r0 = r;

    auto max_iter = static_cast<SizeT>(max_iter_ratio * static_cast<Float>(b.size()));
    max_iter  = std::max(max_iter, SizeT{1});
    auto iter = pcg(x, b, max_iter);

    logger::info("LinearPCG: frame={} newton_iter={} dof={} max_iter={} -> iters={}",
                 engine().frame(),
                 engine().newton_iter(),
                 N,
                 max_iter,
                 iter);

    if(iter >= max_iter)
        logger::warn(
            "LinearPCG: reached max_iter = {} (no early convergence); "
            "check preconditioner or linear_system/tol_rate.",
            max_iter);

    info.iter_count(iter);
}

void LinearPCG::dump_r_z(SizeT k)
{

    auto path_tool   = BackendPathTool(workspace());
    auto output_path = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    auto output_path_r = fmt::format(
        "{}r.{}.{}.{}.mtx", output_path.string(), engine().frame(), engine().newton_iter(), k);

    export_vector_market(output_path_r, r.cview());
    logger::info("Dumped PCG r to {}", output_path_r);

    auto output_path_z = fmt::format(
        "{}z.{}.{}.{}.mtx", output_path.string(), engine().frame(), engine().newton_iter(), k);

    export_vector_market(fmt::format("{}z.{}.{}.{}.mtx",
                                     output_path.string(),
                                     engine().frame(),
                                     engine().newton_iter(),
                                     k),
                         z.cview());

    logger::info("Dumped PCG z to {}", output_path_z);
}

void LinearPCG::dump_p_Ap(SizeT k)
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");

    auto output_path_p = fmt::format("{}p.{}.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter(),
                                     k);

    export_vector_market(output_path_p, p.cview());
    logger::info("Dumped PCG p to {}", output_path_p);

    auto output_path_Ap = fmt::format("{}Ap.{}.{}.{}.mtx",
                                      output_folder.string(),
                                      engine().frame(),
                                      engine().newton_iter(),
                                      k);
    export_vector_market(output_path_Ap, Ap.cview());
    logger::info("Dumped PCG Ap to {}", output_path_Ap);
}

void LinearPCG::check_init_rz_nan_inf(Float rz)
{
    if(!std::isfinite(rz)) [[unlikely]]
    {
        auto norm_r = ctx().norm(r.cview());
        auto norm_z = ctx().norm(z.cview());
        bool r_bad  = !std::isfinite(norm_r);
        auto hint = r_bad ? "gradient assembling produced NaN values, likely due to error in formula implementation" :
                            "preconditioner failed, likely due to inverse matrix calculation failure";
        UIPC_ASSERT(false,
                    "Frame {}, Newton {}, PCG Init: r^T*z = {}, norm(r) = {}, norm(z) = {}. "
                    "Hint: {}.",
                    engine().frame(),
                    engine().newton_iter(),
                    rz,
                    norm_r,
                    norm_z,
                    hint);
    }
}

void LinearPCG::check_iter_rz_nan_inf(Float rz, SizeT k)
{
    if(!std::isfinite(rz)) [[unlikely]]
    {
        auto norm_r = ctx().norm(r.cview());
        auto norm_z = ctx().norm(z.cview());
        bool r_ok   = std::isfinite(norm_r);
        bool z_bad  = !std::isfinite(norm_z);
        auto hint = (r_ok && z_bad) ?
                        "preconditioner failed, likely due to inverse matrix calculation failure" :
                        "PCG iteration diverged";
        UIPC_ASSERT(false,
                    "Frame {}, Newton {}, PCG Iter {}: r^T*z = {}, norm(r) = {}, norm(z) = {}. "
                    "Hint: {}.",
                    engine().frame(),
                    engine().newton_iter(),
                    k,
                    rz,
                    norm_r,
                    norm_z,
                    hint);
    }
}

void update_xr(Float                         alpha,
               muda::DenseVectorView<Float>  x,
               muda::CDenseVectorView<Float> p,
               muda::DenseVectorView<Float>  r,
               muda::CDenseVectorView<Float> Ap)
{
    using namespace muda;

    // Fused update of x and r for better performance
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(r.size(),
               [alpha = alpha,
                x     = x.viewer().name("x"),
                p     = p.cviewer().name("p"),
                r     = r.viewer().name("r"),
                Ap    = Ap.cviewer().name("Ap")] __device__(int i) mutable
               {
                   x(i) += alpha * p(i);
                   r(i) -= alpha * Ap(i);
               });
}

void update_p(muda::DenseVectorView<Float> p, muda::CDenseVectorView<Float> z, Float beta)
{
    using namespace muda;

    // Simple axpby
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(p.size(),
               [p = p.viewer().name("p"), z = z.cviewer().name("z"), beta = beta] __device__(
                   int i) mutable { p(i) = z(i) + beta * p(i); });
}

SizeT LinearPCG::pcg(muda::DenseVectorView<Float> x, muda::CDenseVectorView<Float> b, SizeT max_iter)
{
    Timer pcg_timer{"PCG"};

    SizeT k = 0;
    // r = b - A * x
    {
        // r = b;
        r.buffer_view().copy_from(b.buffer_view());

        // x == 0, so we don't need to do the following
        // r = - A * x + r
        //spmv(-1.0, x.as_const(), 1.0, r.view());
    }

    Float alpha, beta, rz, abs_rz0;

    // z = P * r (apply preconditioner)
    {
        Timer timer{"Apply Preconditioner"};
        apply_preconditioner(z, r, d_converged_false.view());
    }

    if(need_debug_dump) [[unlikely]]
        dump_r_z(k);

    // p = z
    p = z;

    // init rz
    // rz = r^T * z
    rz = ctx().dot(r.cview(), z.cview());
    check_init_rz_nan_inf(rz);

    abs_rz0 = std::abs(rz);

    // check convergence
    if(accuracy_statisfied(r) && abs_rz0 == Float{0.0})
        return 0;

    for(k = 1; k < max_iter; ++k)
    {
        {
            Timer timer{"SpMV"};
            spmv(p.cview(), Ap.view());
        }

        if(need_debug_dump) [[unlikely]]
            dump_p_Ap(k);

        // alpha = rz / p^T * Ap
        alpha = rz / ctx().dot(p.cview(), Ap.cview());

        // x = x + alpha * p
        // r = r - alpha * Ap
        update_xr(alpha, x, p.cview(), r.view(), Ap.cview());

        // z = P * r (apply preconditioner)
        {
            Timer timer{"Apply Preconditioner"};
            apply_preconditioner(z, r, d_converged_false.view());
        }

        if(need_debug_dump) [[unlikely]]
            dump_r_z(k);

        // rz_new = r^T * z
        Float rz_new = ctx().dot(r.cview(), z.cview());
        check_iter_rz_nan_inf(rz_new, k);

        // check convergence
        if(accuracy_statisfied(r) && std::abs(rz_new) <= global_tol_rate * abs_rz0)
            break;

        // beta = rz_new / rz
        beta = rz_new / rz;

        // p = z + beta * p
        update_p(p.view(), z.cview(), beta);

        rz = rz_new;
    }

    return k;
}
}  // namespace uipc::backend::cuda
