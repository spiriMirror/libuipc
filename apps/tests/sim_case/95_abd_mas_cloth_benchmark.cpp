#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_spherical_joint.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <uipc/common/timer.h>
#include <fstream>

// Benchmark: Compare MAS (full hierarchy) vs Diagonal preconditioner on a
// 30×30 grid of affine-body cubes connected by spherical joints ("ABD cloth").
//
// A 2D-connected grid gives far richer inter-cluster coupling than a 1D chain.
// Multi-level MAS should therefore reduce PCG iterations at least 30% vs Diag.

namespace
{
using uipc::Float;
using uipc::IndexT;
using uipc::Vector3;
constexpr int   GRID_N           = 30;    // 30×30 = 900 bodies
constexpr int   BENCHMARK_FRAMES = 20;
// Small grid for algebraic debugging (fits dense numpy eigvals instantly).
constexpr int   SMALL_GRID_N        = 4;   // 4×4 = 16 bodies, 192 DOFs
constexpr int   SMALL_BENCH_FRAMES  = 3;
constexpr Float BODY_SCALE       = 0.1;  // cube half-extent = 0.05
constexpr Float SPACING          = 0.1;  // centre-to-centre = one cube width
constexpr Float HALF_EXT         = BODY_SCALE / 2.0;
constexpr Float STIFFNESS        = 1e8;  // ABD material stiffness
constexpr Float JOINT_STRENGTH   = 1e4;  // default; overridden per-sweep in 95e

// Build the cloth scene without running it (caller controls advance() loop).
// Returns nothing; mutates `scene` in place.
//
// `top_row_strength`:
//   < 0  -> use is_fixed=1 for the top row (default; rigid pin)
//   >= 0 -> use SoftTransformConstraint(translation=top_row_strength, rotation=top_row_strength)
//           to softly hold each top-row body at its initial transform via animator.
void setup_cloth_scene(uipc::core::Scene& scene,
                       int                grid_n,
                       uipc::Float        joint_strength,
                       uipc::Float        top_row_strength = -1.0)
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    AffineBodyConstitution                  abd;
    std::optional<SoftTransformConstraint>  stc_opt;
    if(top_row_strength >= 0)
        stc_opt.emplace();

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(BODY_SCALE);
    SimplicialComplexIO io{pre_transform};

    const int                              total_bodies = grid_n * grid_n;
    vector<S<SimplicialComplexSlot>>       body_slots(total_bodies);
    // For soft constraint: remember each top-row body's initial transform.
    vector<S<core::Object>>                top_row_objects;
    vector<Matrix4x4>                      top_row_initial_transforms;

    for(int j = 0; j < grid_n; j++)
    {
        for(int i = 0; i < grid_n; i++)
        {
            auto obj  = scene.objects().create(fmt::format("body_{}_{}", i, j));
            auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
            abd.apply_to(mesh, STIFFNESS);
            label_surface(mesh);
            Transform t = Transform::Identity();
            t.translate(Vector3{i * SPACING, 0.5, j * SPACING});
            view(mesh.transforms())[0] = t.matrix();

            if(j == 0)
            {
                if(top_row_strength < 0)
                {
                    // Hard constraint via is_fixed flag.
                    auto is_fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
                    view(*is_fixed)[0] = 1;
                }
                else
                {
                    // Soft constraint: translation_strength, rotation_strength
                    stc_opt->apply_to(mesh, Vector2{top_row_strength, top_row_strength});
                    top_row_initial_transforms.push_back(t.matrix());
                }
            }
            auto [slot, _]            = obj->geometries().create(mesh);
            body_slots[i + j * grid_n] = slot;

            if(j == 0 && top_row_strength >= 0)
                top_row_objects.push_back(obj);
        }
    }

    // For soft constraint case: set up animator that holds each top-row body at
    // its initial transform.
    if(top_row_strength >= 0)
    {
        auto& animator = scene.animator();
        for(size_t k = 0; k < top_row_objects.size(); k++)
        {
            auto initial_xform = top_row_initial_transforms[k];
            auto obj           = top_row_objects[k];
            animator.insert(*obj,
                            [initial_xform](core::Animation::UpdateInfo& info)
                            {
                                auto geo_slots = info.geo_slots();
                                auto geo = geo_slots[0]->geometry().as<SimplicialComplex>();
                                auto is_constrained =
                                    geo->instances().find<IndexT>(builtin::is_constrained);
                                view(*is_constrained)[0] = 1;
                                auto aim = geo->instances().find<Matrix4x4>(builtin::aim_transform);
                                view(*aim)[0] = initial_xform;
                            });
        }
    }

    AffineBodySphericalJoint spherical_joint;
    vector<Vector3>                  positions;
    vector<S<SimplicialComplexSlot>> l_slots, r_slots;
    vector<IndexT>                   l_ids, r_ids;
    vector<Float>                    strengths;

    auto add_joint = [&](int bi, int bj, int ri, int rj, Vector3 anchor)
    {
        positions.push_back(anchor);
        l_slots.push_back(body_slots[bi + bj * grid_n]);
        r_slots.push_back(body_slots[ri + rj * grid_n]);
        l_ids.push_back(0);
        r_ids.push_back(0);
        strengths.push_back(joint_strength);
    };
    for(int j = 0; j < grid_n; j++)
        for(int i = 0; i < grid_n - 1; i++)
            add_joint(i, j, i + 1, j,
                      Vector3{i * SPACING + HALF_EXT, 0.5, j * SPACING});
    for(int j = 0; j < grid_n - 1; j++)
        for(int i = 0; i < grid_n; i++)
            add_joint(i, j, i, j + 1,
                      Vector3{i * SPACING, 0.5, j * SPACING + HALF_EXT});

    auto joint_mesh = spherical_joint.create_geometry(span{positions},
                                                      span{l_slots},
                                                      span{l_ids},
                                                      span{r_slots},
                                                      span{r_ids},
                                                      span{strengths});
    auto joint_obj = scene.objects().create("spherical_joints");
    joint_obj->geometries().create(joint_mesh);
}

void run_cloth_scene(const std::string& output_path,
                     bool               use_mas,
                     int                active_levels   = 0,
                     int                grid_n          = GRID_N,
                     int                bench_frames    = BENCHMARK_FRAMES,
                     Float              joint_strength  = JOINT_STRENGTH)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = false;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["linear_system"]["check_interval"] = 1;
    config["dt"]                            = 0.01;

    if(!use_mas)
        config["extras"]["precond"]["force_abd_diag"] = 1;
    if(use_mas)
    {
        config["extras"]["debug"]["dump_mas_matrices"] = 1;
        if(active_levels > 0)
            config["extras"]["precond"]["abd_mas_active_levels"] = active_levels;
    }

    Scene scene{config};
    setup_cloth_scene(scene, grid_n, joint_strength);

    world.init(scene);
    REQUIRE(world.is_valid());

    for(int i = 0; i < bench_frames; i++)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
    }
}
}  // namespace

TEST_CASE("95_abd_mas_cloth_benchmark", "[abd][mas][benchmark]")
{
    using namespace uipc;

    auto base_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Timer::enable_all();

    // ---- Run with MAS preconditioner (full hierarchy) ----
    {
        GlobalTimer mas_timer{"MAS Benchmark"};
        mas_timer.set_as_current();

        auto mas_path = std::string(base_path) + "mas/";
        std::filesystem::create_directories(mas_path);

        run_cloth_scene(mas_path, true);

        auto          json = mas_timer.report_merged_as_json();
        std::ofstream ofs(std::string(base_path) + "timer_mas.json");
        ofs << json.dump(2);
        logger::info("MAS timer saved to {}timer_mas.json", base_path);
    }

    // ---- Run with Diagonal preconditioner ----
    {
        GlobalTimer diag_timer{"Diag Benchmark"};
        diag_timer.set_as_current();

        auto diag_path = std::string(base_path) + "diag/";
        std::filesystem::create_directories(diag_path);

        run_cloth_scene(diag_path, false);

        auto          json = diag_timer.report_merged_as_json();
        std::ofstream ofs(std::string(base_path) + "timer_diag.json");
        ofs << json.dump(2);
        logger::info("Diag timer saved to {}timer_diag.json", base_path);
    }

    Timer::disable_all();

    // ---- Compare results ----
    {
        auto read_json = [](const std::string& path) -> Json
        {
            std::ifstream ifs(path);
            return Json::parse(ifs);
        };

        auto find_timer = [](const Json& j, const std::string& name,
                             auto&& self) -> std::pair<double, int>
        {
            if(j.contains("name") && j["name"].get<std::string>() == name)
            {
                double dur   = j.value("duration", 0.0);
                int    count = j.value("count", 0);
                return {dur, count};
            }
            if(j.contains("children"))
                for(auto& child : j["children"])
                {
                    auto result = self(child, name, self);
                    if(result.second > 0)
                        return result;
                }
            return {0.0, 0};
        };

        auto mas_json  = read_json(std::string(base_path) + "timer_mas.json");
        auto diag_json = read_json(std::string(base_path) + "timer_diag.json");

        auto [mas_spmv_time, mas_spmv_count]   = find_timer(mas_json, "SpMV", find_timer);
        auto [diag_spmv_time, diag_spmv_count] = find_timer(diag_json, "SpMV", find_timer);

        auto [mas_prec_time, mas_prec_count]   = find_timer(mas_json, "Apply Preconditioner", find_timer);
        auto [diag_prec_time, diag_prec_count] = find_timer(diag_json, "Apply Preconditioner", find_timer);

        auto [mas_pcg_time, mas_pcg_count]   = find_timer(mas_json, "FusedPCG", find_timer);
        auto [diag_pcg_time, diag_pcg_count] = find_timer(diag_json, "FusedPCG", find_timer);

        // Newton count = prec_calls - spmv_calls (one extra precond apply at PCG init)
        int mas_newton  = std::max(1, mas_prec_count - mas_spmv_count);
        int diag_newton = std::max(1, diag_prec_count - diag_spmv_count);

        double mas_pcg_per_newton  = static_cast<double>(mas_spmv_count) / mas_newton;
        double diag_pcg_per_newton = static_cast<double>(diag_spmv_count) / diag_newton;
        double ratio               = diag_pcg_per_newton / mas_pcg_per_newton;

        logger::info(
            "===== ABD Cloth PCG Benchmark: {} frames, {}×{} grid ({} bodies) =====",
            BENCHMARK_FRAMES, GRID_N, GRID_N, GRID_N * GRID_N);
        logger::info(
            "  MAS (full hierarchy): PCG/Newton={:.1f}  Newton={}  total_spmv={}",
            mas_pcg_per_newton, mas_newton, mas_spmv_count);
        logger::info(
            "  Diag:                 PCG/Newton={:.1f}  Newton={}  total_spmv={}",
            diag_pcg_per_newton, diag_newton, diag_spmv_count);
        logger::info(
            "  Improvement (Diag/MAS PCG/Newton): {:.2f}x  (target: >= 1.30x)",
            ratio);

        if(mas_pcg_time > 0 && diag_pcg_time > 0)
            logger::info("  FusedPCG time speedup (Diag/MAS): {:.2f}x",
                         diag_pcg_time / mas_pcg_time);

        // 2D grid topology should give ≥30% fewer PCG iters/Newton with full MAS hierarchy
        REQUIRE(mas_pcg_per_newton < diag_pcg_per_newton);
        REQUIRE(ratio >= 1.30);
    }
}

// ---------------------------------------------------------------------------
// Small-grid version for algebraic debugging. Dumps MAS matrices for a 4x4
// cloth (16 bodies, 192 DOFs) which fits dense numpy eigvals instantly.
// ---------------------------------------------------------------------------
TEST_CASE("95c_abd_mas_cloth_small", "[abd][mas][debug]")
{
    using namespace uipc;

    Timer::enable_all();

    auto base_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "small/";
    std::filesystem::create_directories(base_path);

    int spmv_mas = 0, newton_mas = 0;
    int spmv_diag = 0, newton_diag = 0;

    auto run_one = [&](const std::string& name, bool use_mas) -> std::pair<int,int>
    {
        auto path = base_path + name + "/";
        std::filesystem::create_directories(path);
        GlobalTimer t{name};
        t.set_as_current();
        run_cloth_scene(path, use_mas, 1,  // L1 only
                        SMALL_GRID_N, SMALL_BENCH_FRAMES);

        auto json = t.report_merged_as_json();
        int spmv = 0, prec = 0;
        std::function<void(const Json&)> walk = [&](const Json& n)
        {
            if(n.contains("name"))
            {
                auto nm = n["name"].get<std::string>();
                if(nm == "SpMV") spmv = n.value("count", 0);
                if(nm == "Preconditioner") prec = n.value("count", 0);
            }
            if(n.contains("children"))
                for(auto& c : n["children"]) walk(c);
        };
        walk(json);
        return {spmv, prec - spmv};
    };

    auto [sm, nm]   = run_one("mas_L1",   true);
    auto [sd, nd]   = run_one("diag",     false);

    logger::info("===== small 4x4 cloth (16 bodies, {} DOFs) =====",
                 SMALL_GRID_N * SMALL_GRID_N * 12);
    logger::info("  MAS L1: spmv={} newton={} ({}/iter)",
                 sm, nm, nm > 0 ? (double)sm / nm : 0.0);
    logger::info("  Diag:   spmv={} newton={} ({}/iter)",
                 sd, nd, nd > 0 ? (double)sd / nd : 0.0);

    Timer::disable_all();
}

// ---------------------------------------------------------------------------
// Scan grid sizes to find where MAS L1 starts underperforming Diag.
// ---------------------------------------------------------------------------
TEST_CASE("95d_abd_mas_cloth_size_scan", "[abd][mas][debug]")
{
    using namespace uipc;

    Timer::enable_all();

    auto base_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "scan/";
    std::filesystem::create_directories(base_path);

    auto run_size = [&](int grid_n, bool use_mas) -> std::pair<int,int>  // spmv, newton
    {
        auto path = base_path + fmt::format("g{}_{}/", grid_n, use_mas ? "mas" : "diag");
        std::filesystem::create_directories(path);
        GlobalTimer t{fmt::format("size_{}", grid_n)};
        t.set_as_current();
        run_cloth_scene(path, use_mas, 1, grid_n, 10);
        auto json = t.report_merged_as_json();
        int spmv = 0, prec = 0;
        std::function<void(const Json&)> walk = [&](const Json& n)
        {
            if(n.contains("name"))
            {
                auto nm = n["name"].get<std::string>();
                if(nm == "SpMV") spmv = n.value("count", 0);
                if(nm == "Preconditioner") prec = n.value("count", 0);
            }
            if(n.contains("children"))
                for(auto& c : n["children"]) walk(c);
        };
        walk(json);
        return {spmv, std::max(1, prec - spmv)};
    };

    logger::info("===== Cloth size scan (MAS L1 vs Diag) =====");
    for(int n : {4, 6, 8, 10, 12, 16, 20, 24, 30})
    {
        auto [msm, mnt] = run_size(n, true);
        auto [dsm, dnt] = run_size(n, false);
        double mpcg = (double)msm / mnt;
        double dpcg = (double)dsm / dnt;
        logger::info("  {:3d}x{:<3d}: MAS L1={:5d}/{:<3d}n={:.1f}pcg  Diag={:5d}/{:<3d}n={:.1f}pcg  "
                     "(Diag_pcg/MAS_pcg={:.2f}x,  Diag_total/MAS_total={:.2f}x)",
                     n, n, msm, mnt, mpcg, dsm, dnt, dpcg,
                     dpcg / mpcg, (double)dsm / msm);
    }

    Timer::disable_all();
}

// ---------------------------------------------------------------------------
// Joint-strength sweep: test whether MAS multilevel wins over L0 when
// joints are much stiffer than body self-stiffness (stronger global coupling
// should make coarse corrections useful).
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Export the 20x20 ABD cloth scene (joint_k=1e4) to OBJ files for inspection.
// Writes initial frame + 20 advanced frames as scene_surface_NNN.obj.
// ---------------------------------------------------------------------------
TEST_CASE("95g_abd_cloth_export_20x20", "[abd][mas][export]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    const int   grid_n         = 20;
    const int   n_frames       = 20;
    const Float joint_strength = 1e4;

    auto out_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "export_20x20/";
    std::filesystem::create_directories(out_path);

    auto config = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = false;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["linear_system"]["check_interval"] = 1;
    config["dt"]                            = 0.01;

    Engine engine{"cuda", out_path};
    World  world{engine};
    Scene  scene{config};
    setup_cloth_scene(scene, grid_n, joint_strength);
    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface_{:03d}.obj", out_path, world.frame()));

    for(int i = 0; i < n_frames; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface_{:03d}.obj", out_path, world.frame()));
        REQUIRE(world.is_valid());
    }

    logger::info("===== Exported {} frames to {} =====", n_frames + 1, out_path);
}

// ---------------------------------------------------------------------------
// LOCKSTEP comparison: every preconditioner config (Diag / L1 / L2 / Full)
// solves the EXACT SAME Hessian sequence. Pre-records the DOF state from a
// reference DIAG run, then re-runs each config seeded from that state at every
// frame, so all configs visit the same physical states. This eliminates the
// simulation-divergence noise that contaminates the size-scan benchmark.
// ---------------------------------------------------------------------------
TEST_CASE("95f_abd_mas_lockstep_compare", "[abd][mas][debug]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    Timer::enable_all();

    auto base_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "lockstep/";
    std::filesystem::create_directories(base_path);

    const Float joint_strength = 1e4;
    // Set top_row_strength >= 0 to use SoftTransformConstraint instead of is_fixed.
    // Pass -1 to keep the rigid is_fixed pin.
    const Float top_row_strength = 1e8;

    auto run_one_size = [&](int grid_n, int n_frames)
    {
    // ---- Step 1: reference DIAG run; snapshot DOF before each frame ----
    vector<geometry::SimplicialComplex> dof_snapshots;
    {
        auto config = test::Scene::default_config();
        config["gravity"]                       = Vector3{0, -9.8, 0};
        config["contact"]["enable"]             = false;
        config["linear_system"]["solver"]       = std::string{"linear_pcg"};
        config["linear_system"]["tol_rate"]     = 1e-12;  // matches test 94
        config["linear_system"]["check_interval"] = 1;
        config["dt"]                            = 0.01;
        config["extras"]["precond"]["force_abd_diag"] = 1;

        Engine engine{"cuda", base_path + "ref_diag/"};
        World  world{engine};
        Scene  scene{config};
        setup_cloth_scene(scene, grid_n, joint_strength, top_row_strength);
        world.init(scene);
        REQUIRE(world.is_valid());

        auto abd_acc = world.features().find<core::AffineBodyStateAccessorFeature>();
        REQUIRE(abd_acc);

        auto state_geo = abd_acc->create_geometry();
        state_geo.instances().create<Matrix4x4>(builtin::transform);

        abd_acc->copy_to(state_geo);
        dof_snapshots.push_back(state_geo);

        for(int f = 0; f < n_frames; f++)
        {
            world.advance();
            world.retrieve();
            abd_acc->copy_to(state_geo);
            dof_snapshots.push_back(state_geo);
            REQUIRE(world.is_valid());
        }
        logger::info("Reference DIAG: captured {} DOF snapshots", dof_snapshots.size());
    }

    // ---- Step 2: re-run each config seeded from those snapshots ----
    auto run_locked = [&](const std::string& tag, bool use_mas, int active_levels) -> int
    {
        auto config = test::Scene::default_config();
        config["gravity"]                       = Vector3{0, -9.8, 0};
        config["contact"]["enable"]             = false;
        config["linear_system"]["solver"]       = std::string{"linear_pcg"};
        config["linear_system"]["tol_rate"]     = 1e-12;
        config["linear_system"]["check_interval"] = 1;
        config["dt"]                            = 0.01;
        if(!use_mas)
            config["extras"]["precond"]["force_abd_diag"] = 1;
        else
        {
            config["extras"]["debug"]["dump_mas_matrices"] = 1;
            if(active_levels > 0)
                config["extras"]["precond"]["abd_mas_active_levels"] = active_levels;
        }

        auto path = base_path + tag + "/";
        std::filesystem::create_directories(path);
        GlobalTimer t{tag};
        t.set_as_current();

        Engine engine{"cuda", path};
        World  world{engine};
        Scene  scene{config};
        setup_cloth_scene(scene, grid_n, joint_strength, top_row_strength);
        world.init(scene);

        auto abd_acc = world.features().find<core::AffineBodyStateAccessorFeature>();
        REQUIRE(abd_acc);

        for(int f = 0; f < n_frames; f++)
        {
            abd_acc->copy_from(dof_snapshots[f]);
            world.advance();
            world.retrieve();
        }

        auto json = t.report_merged_as_json();
        int spmv = 0;
        std::function<void(const Json&)> walk = [&](const Json& n)
        {
            if(n.contains("name") && n["name"].get<std::string>() == "SpMV")
                spmv = n.value("count", 0);
            if(n.contains("children"))
                for(auto& c : n["children"]) walk(c);
        };
        walk(json);
        return spmv;
    };

    int diag_count = run_locked("diag",      false, 0);
    int l1_count   = run_locked("mas_L1",    true,  1);
    int l2_count   = run_locked("mas_L2",    true,  2);
    int full_count = run_locked("mas_full",  true,  0);

    logger::info("===== LOCKSTEP cloth grid={} joint_k=1e4 frames={} =====",
                 grid_n, n_frames);
    logger::info("  Diag      SpMV = {}", diag_count);
    logger::info("  MAS L1    SpMV = {}    L1/Diag   = {:.3f}",
                 l1_count, double(l1_count) / diag_count);
    logger::info("  MAS L2    SpMV = {}    L2/Diag   = {:.3f}",
                 l2_count, double(l2_count) / diag_count);
    logger::info("  MAS Full  SpMV = {}    Full/Diag = {:.3f}",
                 full_count, double(full_count) / diag_count);
    };

    for(int g : {4, 6, 8, 10, 12, 16, 20})
        run_one_size(g, 5);

    Timer::disable_all();
}

TEST_CASE("95e_abd_mas_joint_sweep", "[abd][mas][debug]")
{
    using namespace uipc;
    Timer::enable_all();

    auto base_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "joint_sweep/";
    std::filesystem::create_directories(base_path);

    const int grid_n = 20;
    const int frames = 10;

    auto run_config = [&](Float joint_k, int active_levels, bool use_mas,
                          const std::string& tag) -> std::pair<int,int>
    {
        auto path = base_path + fmt::format("j{}_a{}_{}/", (int)joint_k, active_levels,
                                            use_mas ? "mas" : "diag");
        std::filesystem::create_directories(path);
        GlobalTimer t{tag};
        t.set_as_current();
        run_cloth_scene(path, use_mas, active_levels, grid_n, frames, joint_k);

        auto json = t.report_merged_as_json();
        int spmv = 0, prec = 0;
        std::function<void(const Json&)> walk = [&](const Json& n)
        {
            if(n.contains("name"))
            {
                auto nm = n["name"].get<std::string>();
                if(nm == "SpMV") spmv = n.value("count", 0);
                if(nm == "Preconditioner") prec = n.value("count", 0);
            }
            if(n.contains("children"))
                for(auto& c : n["children"]) walk(c);
        };
        walk(json);
        return {spmv, std::max(1, prec - spmv)};
    };

    logger::info("===== Joint strength sweep ({}x{} cloth, {} frames) =====",
                 grid_n, grid_n, frames);
    for(Float jk : {1e4, 1e5, 1e6})
    {
        auto [diag_spmv, diag_n] = run_config(jk, 1, false, "diag");
        auto [l1_spmv,   l1_n]   = run_config(jk, 1, true,  "L1");
        auto [full_spmv, full_n] = run_config(jk, 0, true,  "full");
        logger::info(
            "  joint_k={:.0e}: Diag={:5d}  L1={:5d}  Full={:5d}  "
            "(L1/Diag={:.2f}  Full/Diag={:.2f}  Full/L1={:.2f})",
            jk, diag_spmv, l1_spmv, full_spmv,
            (double)l1_spmv   / diag_spmv,
            (double)full_spmv / diag_spmv,
            (double)full_spmv / l1_spmv);
    }
    Timer::disable_all();
}

// ---------------------------------------------------------------------------
// Level-by-level sweep for the cloth scene: see how each coarse level
// contributes algebraically.
// ---------------------------------------------------------------------------
TEST_CASE("95b_abd_mas_cloth_level_sweep", "[abd][mas][benchmark]")
{
    using namespace uipc;

    Timer::enable_all();

    auto base_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "sweep/";
    std::filesystem::create_directories(base_path);

    auto run_level = [&](int lvl) -> std::pair<int,int>
    {
        auto path = base_path + fmt::format("lvl{}/", lvl);
        std::filesystem::create_directories(path);
        GlobalTimer t{fmt::format("level{}", lvl)};
        t.set_as_current();
        run_cloth_scene(path, true, lvl);

        auto json = t.report_merged_as_json();
        int spmv = 0, prec = 0;
        std::function<void(const Json&)> walk = [&](const Json& n)
        {
            if(n.contains("name"))
            {
                auto name = n["name"].get<std::string>();
                if(name == "SpMV") spmv = n.value("count", 0);
                if(name == "Preconditioner") prec = n.value("count", 0);
            }
            if(n.contains("children"))
                for(auto& c : n["children"]) walk(c);
        };
        walk(json);
        return {spmv, prec - spmv};
    };

    logger::info("===== ABD Cloth MAS Level Sweep: {}x{} grid ({} bodies) =====",
                 GRID_N, GRID_N, GRID_N * GRID_N);
    for(int lvl : {1, 2, 3, 4, 0})
    {
        auto [spmv, newton] = run_level(lvl);
        double pcg_per_newton = newton > 0 ? (double)spmv / newton : 0.0;
        logger::info("  level {:2d}  spmv={:6d}  newton={:4d}  PCG/Newton={:.1f}",
                     lvl, spmv, newton, pcg_per_newton);
    }
    Timer::disable_all();
}
