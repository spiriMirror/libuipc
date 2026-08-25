#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>

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
    REQUIRE(entries.at("newton/use_adaptive_tol").at("status") == "reserved");
    REQUIRE(entries.at("newton/use_adaptive_tol").at("const") == 0);
    REQUIRE(entries.at("sanity_check/mode").at("enum") == Json::array({"normal", "quiet"}));

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
