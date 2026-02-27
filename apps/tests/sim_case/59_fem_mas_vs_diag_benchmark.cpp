#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/common/timer.h>
#include <fstream>

// Benchmark: Compare MAS vs Diagonal preconditioner on the same bunny scene.
// Runs N frames with each, records Timer JSON to files for comparison.

namespace
{
constexpr int BENCHMARK_FRAMES = 20;

void run_bunny_scene(const std::string& output_path, bool use_mas)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;

    Scene scene{config};
    {
        StableNeoHookean snh;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();

        auto object = scene.objects().create("bunny");

        SimplicialComplexIO io;
        auto bunny = io.read(fmt::format("{}/bunny0.msh", tetmesh_dir));

        label_surface(bunny);
        label_triangle_orient(bunny);

        if(use_mas)
            mesh_partition(bunny, 16);

        // Stiff material — MAS shines more with higher stiffness
        auto parm = ElasticModuli::youngs_poisson(10.0_MPa, 0.49);
        snh.apply_to(bunny, parm);
        default_element.apply_to(bunny);

        object->geometries().create(bunny);

        auto g = ground(-1.0);
        object->geometries().create(g);
    }

    world.init(scene);

    for(int i = 0; i < BENCHMARK_FRAMES; i++)
    {
        world.advance();
        world.retrieve();
    }
}
}  // namespace

TEST_CASE("59_fem_mas_vs_diag_benchmark", "[fem][mas][benchmark]")
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

        run_bunny_scene(mas_path, true);

        auto json = mas_timer.report_merged_as_json();
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

        run_bunny_scene(diag_path, false);

        auto json = diag_timer.report_merged_as_json();
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

        auto [mas_prec_time, mas_prec_count]   = find_timer(mas_json, "Apply Preconditioner", find_timer);
        auto [diag_prec_time, diag_prec_count] = find_timer(diag_json, "Apply Preconditioner", find_timer);

        auto [mas_spmv_time, mas_spmv_count]   = find_timer(mas_json, "SpMV", find_timer);
        auto [diag_spmv_time, diag_spmv_count] = find_timer(diag_json, "SpMV", find_timer);

        logger::info("===== PCG Benchmark: {} frames =====", BENCHMARK_FRAMES);
        logger::info("  MAS:  PCG total={:.3f}s, calls={}, spmv={:.3f}s ({} calls), precond={:.3f}s ({} calls)",
                     mas_pcg_time, mas_pcg_count,
                     mas_spmv_time, mas_spmv_count,
                     mas_prec_time, mas_prec_count);
        logger::info("  Diag: PCG total={:.3f}s, calls={}, spmv={:.3f}s ({} calls), precond={:.3f}s ({} calls)",
                     diag_pcg_time, diag_pcg_count,
                     diag_spmv_time, diag_spmv_count,
                     diag_prec_time, diag_prec_count);

        if(diag_spmv_count > 0 && mas_spmv_count > 0)
        {
            logger::info("  SpMV calls ratio (Diag/MAS): {:.2f}x",
                         static_cast<double>(diag_spmv_count) / mas_spmv_count);
        }
        if(diag_prec_count > 0 && mas_prec_count > 0)
        {
            logger::info("  Preconditioner calls ratio (Diag/MAS): {:.2f}x",
                         static_cast<double>(diag_prec_count) / mas_prec_count);
        }
        if(diag_pcg_time > 0 && mas_pcg_time > 0)
        {
            logger::info("  PCG time speedup (Diag/MAS): {:.2f}x",
                         diag_pcg_time / mas_pcg_time);
        }

        // MAS must produce fewer SpMV calls (= fewer PCG iterations) than Diagonal
        REQUIRE(mas_spmv_count < diag_spmv_count);
    }
}
