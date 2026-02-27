#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/timer.h>
#include <fstream>

// Benchmark: Compare contact-aware MAS ON vs OFF on a self-colliding cloth.
// A large cloth (80x80) fixed at the center vertex, draping under gravity
// so that it folds onto itself, producing self-collision.

namespace
{
constexpr int BENCHMARK_FRAMES = 50;

void run_cloth_self_contact(const std::string& output_path, bool contact_aware)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string trimesh_dir{AssetDir::trimesh_path()};

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;

    // Toggle contact-aware
    config["linear_system"]["precond"]["mas"]["contact_aware"] = contact_aware;

    Scene scene{config};
    {
        NeoHookeanShell nhs;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();

        auto object = scene.objects().create("cloth");

        SimplicialComplexIO io;
        auto cloth = io.read(fmt::format("{}cloth80x80.obj", trimesh_dir));
        label_surface(cloth);

        // Partition for MAS
        mesh_partition(cloth, 16);

        auto parm = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49);
        nhs.apply_to(cloth, parm);
        default_element.apply_to(cloth);

        // Fix the center vertex so cloth drapes and folds onto itself
        {
            auto  positions = cloth.positions().view();
            SizeT num_verts = positions.size();

            // Find vertex closest to the centroid
            Vector3 centroid = Vector3::Zero();
            for(auto& p : positions)
                centroid += p;
            centroid /= static_cast<Float>(num_verts);

            SizeT  center_idx  = 0;
            Float  min_dist_sq = std::numeric_limits<Float>::max();
            for(SizeT i = 0; i < num_verts; i++)
            {
                Float d = (positions[i] - centroid).squaredNorm();
                if(d < min_dist_sq)
                {
                    min_dist_sq = d;
                    center_idx  = i;
                }
            }

            auto is_fixed      = cloth.vertices().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);
            is_fixed_view[center_idx] = 1;
        }

        object->geometries().create(cloth);
    }

    world.init(scene);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < BENCHMARK_FRAMES)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
}  // namespace

TEST_CASE("62_fem_mas_contact_aware_benchmark", "[fem][mas][benchmark]")
{
    using namespace uipc;

    auto base_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Timer::enable_all();

    // ---- Run with contact_aware ON ----
    int on_spmv_count = 0;
    {
        GlobalTimer on_timer{"ContactAware ON"};
        on_timer.set_as_current();

        auto path = std::string(base_path) + "on/";
        std::filesystem::create_directories(path);
        run_cloth_self_contact(path, true);

        auto json = on_timer.report_merged_as_json();
        std::ofstream ofs(std::string(base_path) + "timer_ca_on.json");
        ofs << json.dump(2);

        auto find_timer = [](const Json& j, const std::string& name,
                             auto&& self) -> std::pair<double, int>
        {
            if(j.contains("name") && j["name"].get<std::string>() == name)
                return {j.value("duration", 0.0), j.value("count", 0)};
            if(j.contains("children"))
                for(auto& child : j["children"])
                {
                    auto r = self(child, name, self);
                    if(r.second > 0) return r;
                }
            return {0.0, 0};
        };

        auto [t, c] = find_timer(json, "SpMV", find_timer);
        on_spmv_count = c;
    }

    // ---- Run with contact_aware OFF ----
    int off_spmv_count = 0;
    {
        GlobalTimer off_timer{"ContactAware OFF"};
        off_timer.set_as_current();

        auto path = std::string(base_path) + "off/";
        std::filesystem::create_directories(path);
        run_cloth_self_contact(path, false);

        auto json = off_timer.report_merged_as_json();
        std::ofstream ofs(std::string(base_path) + "timer_ca_off.json");
        ofs << json.dump(2);

        auto find_timer = [](const Json& j, const std::string& name,
                             auto&& self) -> std::pair<double, int>
        {
            if(j.contains("name") && j["name"].get<std::string>() == name)
                return {j.value("duration", 0.0), j.value("count", 0)};
            if(j.contains("children"))
                for(auto& child : j["children"])
                {
                    auto r = self(child, name, self);
                    if(r.second > 0) return r;
                }
            return {0.0, 0};
        };

        auto [t, c] = find_timer(json, "SpMV", find_timer);
        off_spmv_count = c;
    }

    Timer::disable_all();

    logger::info("===== Contact-Aware Benchmark: {} frames, self-colliding cloth =====",
                 BENCHMARK_FRAMES);
    logger::info("  ON:  SpMV calls = {}", on_spmv_count);
    logger::info("  OFF: SpMV calls = {}", off_spmv_count);
    if(off_spmv_count > 0 && on_spmv_count > 0)
    {
        logger::info("  Ratio OFF/ON: {:.2f}x",
                     static_cast<double>(off_spmv_count) / on_spmv_count);
    }

    // Contact-aware should not significantly regress.
    // Allow 5% tolerance since hierarchy changes can shift Newton convergence slightly.
    REQUIRE(on_spmv_count <= static_cast<int>(off_spmv_count * 1.05));
}
