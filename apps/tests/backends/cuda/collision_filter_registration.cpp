#include <app/app.h>
#include <uipc/uipc.h>

namespace
{
std::string_view expected_filter_type(std::string_view method)
{
    if(method == "info_stackless_bvh")
        return "InfoStacklessBVHSimplexTrajectoryFilter";
    if(method == "info_stackless_bvh_v0")
        return "InfoStacklessBVHV0SimplexTrajectoryFilter";
    if(method == "stackless_bvh")
        return "StacklessBVHSimplexTrajectoryFilter";
    if(method == "linear_bvh")
        return "LBVHSimplexTrajectoryFilter";
    return {};
}
}  // namespace

TEST_CASE("compiled collision filters match the scene schema", "[cuda][collision detection][config]")
{
    using namespace uipc;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    const auto schema = Scene::config_schema();
    const auto methods = schema.at("entries").at("collision_detection/method").at("enum");

    for(const auto& method_json : methods)
    {
        const auto method   = method_json.get<std::string>();
        const auto expected = expected_filter_type(method);
        CAPTURE(method);
        CAPTURE(expected);
        REQUIRE_FALSE(expected.empty());

        const auto workspace =
            fs::path{AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)} / method;
        Engine engine{"cuda", workspace.string()};
        World  world{engine};

        auto config                             = Scene::default_config();
        config["sanity_check"]["enable"]        = false;
        config["collision_detection"]["method"] = method;
        Scene scene{config};
        world.init(scene);
        REQUIRE(world.is_valid());

        const auto engine_json = engine.to_json();
        bool       found       = false;
        for(const auto& system : engine_json.at("sim_systems"))
        {
            const auto name = system.at("name").get<std::string>();
            found |= name.find(expected) != std::string::npos;
        }
        REQUIRE(found);
    }
}
