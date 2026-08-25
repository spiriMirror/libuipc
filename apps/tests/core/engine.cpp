#include <app/app.h>
#include <app/asset_dir.h>

#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/core/i_engine.h>
#include <fstream>

namespace
{
class CountingEngine final : public uipc::core::IEngine
{
  public:
    uipc::SizeT advance_count  = 0;
    uipc::SizeT sync_count     = 0;
    uipc::SizeT retrieve_count = 0;

  private:
    void do_init(uipc::core::internal::World&) override {}

    void do_advance() override
    {
        ++advance_count;
        ++m_frame;
    }

    void do_sync() override { ++sync_count; }

    void do_retrieve() override { ++retrieve_count; }

    uipc::SizeT get_frame() const override { return m_frame; }

    uipc::core::EngineStatusCollection& get_status() override
    {
        return m_status;
    }

    const uipc::core::FeatureCollection& get_features() const override
    {
        return m_features;
    }

    uipc::SizeT                        m_frame = 0;
    uipc::core::EngineStatusCollection m_status;
    uipc::core::FeatureCollection      m_features;
};
}  // namespace

void test_engine(std::string_view name)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    auto this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{name, this_output_path};
    World  world{engine};

    auto config                      = Scene::default_config();
    config["sanity_check"]["enable"] = 0;
    Scene scene{config};

    AffineBodyConstitution abd;
    scene.constitution_tabular().insert(abd);

    auto& contact_tabular = scene.contact_tabular();
    auto  default_contact = contact_tabular.default_element();


    auto object = scene.objects().create("cube");

    SimplicialComplexIO io;
    auto mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    // create 5 instances of the mesh, share the underlying mesh
    mesh.instances().resize(5);

    // apply the default contact information to the geometry
    default_contact.apply_to(mesh);

    // apply the constitution to the geometry
    // all the instances will have the same constitution
    abd.apply_to(mesh, 1e8);

    // copy_from the mesh to the object
    // to create the geometry and the rest geometry for simulation
    auto [geo, rest_geo] = object->geometries().create(mesh);

    // initialize the world using the scene
    world.init(scene);
    REQUIRE(engine.frame_stats() == Json::object());

    //std::string output_path = fmt::format("{}/{}", this_output_path, name);

    //fs::exists(output_path) || fs::create_directories(output_path);

    //{
    //    std::ofstream f(fmt::format("{}/engine.json", output_path, name));
    //    f << engine.to_json().dump(4);
    //}

    //std::size_t total_frames = 1;

    //// main loop
    //while(world.frame() < total_frames)
    //{
    //    world.advance();
    //    world.retrieve();
    //}
}


TEST_CASE("engine", "[world]")
{
    test_engine("none");
}

TEST_CASE("retrieve synchronizes at most once per advance", "[world]")
{
    using namespace uipc;
    using namespace uipc::core;

    auto   output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    auto   backend     = make_shared<CountingEngine>();
    Engine engine{"counting", backend, output_path};
    World  world{engine};

    auto config                      = Scene::default_config();
    config["sanity_check"]["enable"] = 0;
    Scene scene{config};
    world.init(scene);
    REQUIRE(engine.frame_stats() == Json::object());

    world.retrieve();
    world.retrieve();
    REQUIRE(backend->sync_count == 1);
    REQUIRE(backend->retrieve_count == 2);

    world.advance();
    world.retrieve();
    REQUIRE(backend->advance_count == 1);
    REQUIRE(backend->sync_count == 2);

    world.sync();
    world.retrieve();
    REQUIRE(backend->sync_count == 3);
    REQUIRE(backend->retrieve_count == 4);
}

TEST_CASE("none backend settles pending scene mutations on advance", "[world]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    auto   output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    Engine engine{"none", output_path};
    World  world{engine};

    auto config                      = Scene::default_config();
    config["sanity_check"]["enable"] = 0;
    Scene scene{config};
    world.init(scene);

    backend::SceneVisitor visitor{scene};
    REQUIRE(visitor.is_pending());

    std::vector<Vector3> positions = {Vector3::Zero()};
    auto                 mesh      = pointcloud(positions);
    auto                 object    = scene.objects().create("point");
    object->geometries().create(mesh);

    REQUIRE(visitor.geometries().empty());
    REQUIRE(visitor.pending_geometries().size() == 1);
    REQUIRE(visitor.pending_rest_geometries().size() == 1);

    world.advance();

    REQUIRE_FALSE(visitor.is_pending());
    REQUIRE(visitor.geometries().size() == 1);
    REQUIRE(visitor.rest_geometries().size() == 1);
    REQUIRE(visitor.pending_geometries().empty());
    REQUIRE(visitor.pending_rest_geometries().empty());

    scene.objects().destroy(object->id());
    REQUIRE(visitor.pending_destroy_ids().size() == 1);

    world.advance();

    REQUIRE(visitor.geometries().empty());
    REQUIRE(visitor.rest_geometries().empty());
    REQUIRE(visitor.pending_destroy_ids().empty());
}
