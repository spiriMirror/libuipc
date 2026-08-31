#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>
#include <algorithm>

TEST_CASE("scene_config_schema_and_validation", "[scene][config]")
{
    using namespace uipc;
    using namespace uipc::core;

    const auto  schema  = Scene::config_schema();
    const auto& entries = schema.at("entries");

    REQUIRE(schema.at("schemaVersion") == 1);
    REQUIRE(schema.at("strictUnknownKeys") == true);
    REQUIRE(entries.size() == 46);
    REQUIRE(entries.at("dt").at("default").get<Float>() == Catch::Approx(0.01));
    REQUIRE(entries.at("dt").at("unit") == "s");
    REQUIRE(entries.at("gravity").at("componentCount") == 3);
    REQUIRE(entries.at("newton/semi_implicit/enable").at("default").get<IndexT>() == 1);
    REQUIRE(entries.at("newton/semi_implicit/K_min").at("default").get<IndexT>() == 6);
    REQUIRE(entries.at("newton/use_adaptive_tol").at("status") == "reserved");
    REQUIRE(entries.at("newton/use_adaptive_tol").at("const") == 0);
    REQUIRE(entries.at("sanity_check/mode").at("enum") == Json::array({"normal", "quiet"}));

    const auto& collision          = entries.at("collision_detection/method");
    const auto& methods            = collision.at("enum");
    const auto& conditional_values = collision.at("conditionalValues");
    const bool legacy_collision_built = conditional_values.at("enabled").get<bool>();
    REQUIRE(methods.front() == "info_stackless_bvh");
    REQUIRE(conditional_values.at("backend") == "cuda");
    REQUIRE(conditional_values.at("buildOption") == "UIPC_WITH_CUDA_LEGACY_COLLISION");
    for(const auto& legacy_method : conditional_values.at("values"))
    {
        const bool listed = std::ranges::find(methods, legacy_method) != methods.end();
        REQUIRE(listed == legacy_collision_built);
    }

    for(auto&& [name, entry] : entries.items())
    {
        CAPTURE(name);
        REQUIRE(entry.contains("default"));
        REQUIRE(entry.contains("type"));
        REQUIRE(entry.contains("storageType"));
        REQUIRE(entry.contains("description"));
        REQUIRE(entry.contains("consumers"));
        REQUIRE(entry.contains("lifecycle"));
        REQUIRE(entry.contains("status"));
    }

    SECTION("constructor constraints")
    {
        auto config  = Scene::default_config();
        config["dt"] = 0.0;
        REQUIRE_THROWS(Scene{config});

        config                       = Scene::default_config();
        config["integrator"]["type"] = "rk4";
        REQUIRE_THROWS(Scene{config});

        config                               = Scene::default_config();
        config["newton"]["use_adaptive_tol"] = 1;
        REQUIRE_THROWS(Scene{config});

        config                                  = Scene::default_config();
        config["collision_detection"]["method"] = "linear_bvh";
        if(legacy_collision_built)
            REQUIRE_NOTHROW(Scene{config});
        else
            REQUIRE_THROWS(Scene{config});

        config                                      = Scene::default_config();
        config["contact"]["adaptive"]["min_kappa"]  = 2.0e9;
        config["contact"]["adaptive"]["init_kappa"] = 1.0e9;
        REQUIRE_THROWS(Scene{config});
    }

    SECTION("mutable config is revalidated")
    {
        Scene scene;
        auto  dt = scene.config().find<Float>("dt");
        REQUIRE(dt);
        view(*dt)[0] = -1.0;
        REQUIRE_THROWS(scene.validate_config());
    }
}

TEST_CASE("tabular_extension_configs_are_not_silently_ignored", "[scene][config]")
{
    using namespace uipc;
    using namespace uipc::core;

    Scene scene;

    auto& contact   = scene.contact_tabular();
    auto  contact_a = contact.create("a");
    auto  contact_b = contact.create("b");
    REQUIRE_THROWS(
        contact.insert(contact_a, contact_b, 0.5, 1.0e8, true, Json{{"ignored", 1}}));
    REQUIRE_THROWS(contact.default_model(0.5, 1.0e8, true, Json{{"ignored", 1}}));

    auto& subscene   = scene.subscene_tabular();
    auto  subscene_a = subscene.create("a");
    auto  subscene_b = subscene.create("b");
    REQUIRE_THROWS(subscene.insert(subscene_a, subscene_b, true, Json{{"ignored", 1}}));
}
