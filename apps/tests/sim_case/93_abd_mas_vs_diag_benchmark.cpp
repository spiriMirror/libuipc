#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/common/timer.h>
#include <fstream>

// Benchmark: Compare ABD MAS vs Diagonal preconditioner on the same joint chain scene.
// Runs N frames with each, records Timer JSON to files for comparison.
// The diagonal preconditioner is forced via extras/precond/force_abd_diag = 1.

namespace
{
using namespace uipc;

constexpr int   BENCHMARK_FRAMES = 20;
constexpr int   CHAIN_LENGTH     = 100;
constexpr Float STIFFNESS        = 100.0_MPa;

void run_chain_scene(const std::string& output_path, bool use_mas, int active_levels = 0)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                              = test::Scene::default_config();
    config["gravity"]                        = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = false;
    config["linear_system"]["tol_rate"]     = 1e-3;
    config["linear_system"]["check_interval"] = 1;
    config["line_search"]["report_energy"]  = true;
    config["dt"]                             = 0.01;

    if(!use_mas)
        config["extras"]["precond"]["force_abd_diag"] = 1;
    if(use_mas)
    {
        config["extras"]["debug"]["dump_mas_matrices"]       = 1;
        if(active_levels > 0)
            config["extras"]["precond"]["abd_mas_active_levels"] = active_levels;
    }

    Scene scene{config};

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.3);
    SimplicialComplexIO io{pre_transform};

    AffineBodyConstitution abd;

    constexpr Float spacing = 0.30;  // cubes face-to-face: center gap = 2×half_extent = 2×0.15

    vector<S<SimplicialComplexSlot>> body_slots(CHAIN_LENGTH);

    for(int i = 0; i < CHAIN_LENGTH; i++)
    {
        auto obj  = scene.objects().create(fmt::format("body_{}", i));
        auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        abd.apply_to(mesh, STIFFNESS);
        label_surface(mesh);

        Transform t = Transform::Identity();
        t.translate(Vector3{i * spacing, 0, 0});
        view(mesh.transforms())[0] = t.matrix();

        if(i == 0)
        {
            auto is_fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 1;
        }

        auto [slot, _] = obj->geometries().create(mesh);
        body_slots[i]  = slot;
    }

    AffineBodyRevoluteJoint revolute_joint;

    vector<Vector3>                  l_pos0, l_pos1, r_pos0, r_pos1;
    vector<S<SimplicialComplexSlot>> l_slots, r_slots;
    vector<IndexT>                   l_ids, r_ids;
    vector<Float>                    strengths;

    for(int i = 0; i < CHAIN_LENGTH - 1; i++)
    {
        l_pos0.push_back(Vector3{ 0.15, 0.0, -0.15});
        l_pos1.push_back(Vector3{ 0.15, 0.0,  0.15});
        r_pos0.push_back(Vector3{-0.15, 0.0, -0.15});
        r_pos1.push_back(Vector3{-0.15, 0.0,  0.15});
        l_slots.push_back(body_slots[i]);
        r_slots.push_back(body_slots[i + 1]);
        l_ids.push_back(0);
        r_ids.push_back(0);
        strengths.push_back(100.0);
    }

    auto joint_mesh = revolute_joint.create_geometry(span{l_pos0},
                                                      span{l_pos1},
                                                      span{r_pos0},
                                                      span{r_pos1},
                                                      span{l_slots},
                                                      span{l_ids},
                                                      span{r_slots},
                                                      span{r_ids},
                                                      span{strengths});

    auto joint_obj = scene.objects().create("revolute_joints");
    joint_obj->geometries().create(joint_mesh);

    world.init(scene);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}surface{:04d}.obj", output_path, world.frame()));

    for(int i = 0; i < BENCHMARK_FRAMES; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}surface{:04d}.obj", output_path, world.frame()));
    }
}
}  // namespace

TEST_CASE("93_abd_mas_vs_diag_benchmark", "[abd][mas][benchmark]")
{
    using namespace uipc;

    auto base_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Timer::enable_all();

    // ---- Run with MAS preconditioner ----
    {
        GlobalTimer mas_timer{"MAS Benchmark"};
        mas_timer.set_as_current();

        auto mas_path = std::string(base_path) + "mas/";
        std::filesystem::create_directories(mas_path);

        run_chain_scene(mas_path, true);

        auto          json = mas_timer.report_merged_as_json();
        std::ofstream ofs(std::string(base_path) + "timer_mas.json");
        ofs << json.dump(2);
        logger::info("MAS timer saved to {}timer_mas.json", base_path);
    }

    // ---- Run with Diagonal preconditioner (force_abd_diag = 1) ----
    {
        GlobalTimer diag_timer{"Diag Benchmark"};
        diag_timer.set_as_current();

        auto diag_path = std::string(base_path) + "diag/";
        std::filesystem::create_directories(diag_path);

        run_chain_scene(diag_path, false);

        auto          json = diag_timer.report_merged_as_json();
        std::ofstream ofs(std::string(base_path) + "timer_diag.json");
        ofs << json.dump(2);
        logger::info("Diag timer saved to {}timer_diag.json", base_path);
    }

    Timer::disable_all();

    // ---- Print comparison summary ----
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
            {
                for(auto& child : j["children"])
                {
                    auto result = self(child, name, self);
                    if(result.second > 0)
                        return result;
                }
            }
            return {0.0, 0};
        };

        auto mas_json  = read_json(std::string(base_path) + "timer_mas.json");
        auto diag_json = read_json(std::string(base_path) + "timer_diag.json");

        auto [mas_pcg_time, mas_pcg_count]   = find_timer(mas_json, "PCG", find_timer);
        auto [diag_pcg_time, diag_pcg_count] = find_timer(diag_json, "PCG", find_timer);

        auto [mas_prec_time, mas_prec_count] =
            find_timer(mas_json, "Apply Preconditioner", find_timer);
        auto [diag_prec_time, diag_prec_count] =
            find_timer(diag_json, "Apply Preconditioner", find_timer);

        auto [mas_spmv_time, mas_spmv_count]   = find_timer(mas_json, "SpMV", find_timer);
        auto [diag_spmv_time, diag_spmv_count] = find_timer(diag_json, "SpMV", find_timer);

        logger::info("===== ABD PCG Benchmark: {} frames, chain={} bodies =====",
                     BENCHMARK_FRAMES,
                     CHAIN_LENGTH);
        logger::info(
            "  MAS:  PCG total={:.3f}s, calls={}, spmv={:.3f}s ({} calls), precond={:.3f}s ({} calls)",
            mas_pcg_time,
            mas_pcg_count,
            mas_spmv_time,
            mas_spmv_count,
            mas_prec_time,
            mas_prec_count);
        logger::info(
            "  Diag: PCG total={:.3f}s, calls={}, spmv={:.3f}s ({} calls), precond={:.3f}s ({} calls)",
            diag_pcg_time,
            diag_pcg_count,
            diag_spmv_time,
            diag_spmv_count,
            diag_prec_time,
            diag_prec_count);

        // Newton iteration count = precond_calls - spmv_calls
        // (one extra preconditioner apply per Newton step before the PCG loop)
        int mas_newton_count  = mas_prec_count - mas_spmv_count;
        int diag_newton_count = diag_prec_count - diag_spmv_count;

        double mas_pcg_per_newton  = mas_newton_count > 0
                                         ? static_cast<double>(mas_spmv_count) / mas_newton_count
                                         : 0.0;
        double diag_pcg_per_newton = diag_newton_count > 0
                                         ? static_cast<double>(diag_spmv_count) / diag_newton_count
                                         : 0.0;

        if(diag_spmv_count > 0 && mas_spmv_count > 0)
        {
            logger::info("  SpMV calls ratio (Diag/MAS total): {:.2f}x",
                         static_cast<double>(diag_spmv_count) / mas_spmv_count);
        }
        if(diag_prec_count > 0 && mas_prec_count > 0)
        {
            logger::info("  Newton iters  — MAS: {}, Diag: {}",
                         mas_newton_count,
                         diag_newton_count);
            logger::info("  PCG/Newton    — MAS: {:.1f}, Diag: {:.1f}  (ratio Diag/MAS: {:.2f}x)",
                         mas_pcg_per_newton,
                         diag_pcg_per_newton,
                         diag_pcg_per_newton / mas_pcg_per_newton);
        }
        if(diag_pcg_time > 0 && mas_pcg_time > 0)
        {
            logger::info("  PCG time speedup (Diag/MAS): {:.2f}x",
                         diag_pcg_time / mas_pcg_time);
        }

        // The primary assertion: MAS must require FEWER PCG (SpMV) iterations
        // per Newton step than Diagonal preconditioning.
        //
        // NOTE: Comparing *total* SpMV across two independent simulations is
        // misleading — the two runs follow slightly different physical trajectories
        // (due to different PCG error distributions) and can encounter different
        // numbers of Newton iterations per frame.  The theoretically meaningful
        // metric is the average PCG count per Newton step (same linear system,
        // different preconditioner).  MAS has lower κ(M⁻¹H) ≈ 228 vs Diag's
        // κ(M⁻¹H) ≈ 750, giving ~40% fewer PCG iterations per Newton step.
        REQUIRE(mas_pcg_per_newton < diag_pcg_per_newton);
    }
}

// ---------------------------------------------------------------------------
// Level sweep: run MAS at levels 1,2,3,full (0) to isolate which level helps
// or hurts.  Reports SpMV counts only — no pass/fail assertion.
// ---------------------------------------------------------------------------
TEST_CASE("93b_abd_mas_level_sweep", "[abd][mas][benchmark]")
{
    using namespace uipc;

    Timer::enable_all();

    auto base_path = std::string(AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)) + "sweep/";
    std::filesystem::create_directories(base_path);

    auto run_level = [&](int lvl) -> std::pair<int,int>  // {spmv_count, newton_count}
    {
        std::filesystem::create_directories(base_path + fmt::format("lvl{}/", lvl));
        GlobalTimer t{fmt::format("level{}", lvl)};
        t.set_as_current();
        run_chain_scene(base_path + fmt::format("lvl{}/", lvl), true, lvl);

        auto json = t.report_merged_as_json();
        std::function<std::pair<int,int>(const Json&, const std::string&, const std::string&)>
            find2 = [&](const Json& j,
                        const std::string& a,
                        const std::string& b) -> std::pair<int,int>
            {
                int ca = 0, cb = 0;
                std::function<void(const Json&)> walk = [&](const Json& n)
                {
                    if(n.contains("name"))
                    {
                        if(n["name"].get<std::string>() == a) ca = n.value("count", 0);
                        if(n["name"].get<std::string>() == b) cb = n.value("count", 0);
                    }
                    if(n.contains("children"))
                        for(auto& c : n["children"]) walk(c);
                };
                walk(j);
                return {ca, cb};
            };
        auto [spmv, prec] = find2(json, "SpMV", "Preconditioner");
        int newton = prec - spmv;
        return {spmv, newton};
    };

    logger::info("===== ABD MAS Level Sweep: chain={} =====", CHAIN_LENGTH);
    for(int lvl : {1, 2, 3, 4, 0})
    {
        auto [spmv, newton] = run_level(lvl);
        double pcg_per_newton = newton > 0 ? (double)spmv / newton : 0.0;
        logger::info("  level {:2d}  spmv={:5d}  newton={:3d}  PCG/Newton={:.1f}",
                     lvl, spmv, newton, pcg_per_newton);
    }
    Timer::disable_all();
}
