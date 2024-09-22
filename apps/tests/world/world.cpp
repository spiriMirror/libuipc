#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

using namespace uipc;
using namespace uipc::world;
using namespace uipc::geometry;
using namespace uipc::engine;
using namespace uipc::constitution;

class NullEngine : public IEngine
{
    std::size_t frame = 0;

  public:
    void do_init(backend::WorldVisitor v) override {}
    void do_advance() override
    {
        frame++;
        spdlog::info("[NullEngine] frame: {}", frame);
    }
    void do_sync() override {}
    void do_retrieve() override {}

    // Inherited via IEngine
    bool  do_dump() override { return false; }
    bool  do_recover() override { return false; }
    SizeT get_frame() const override { return frame; }
};

TEST_CASE("simple_world", "[world]")
{
    NullEngine engine;
    World      world{engine};

    Scene scene;

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
    abd.apply_to(mesh, 100.0_MPa);

    // copy_from the mesh to the object
    // to create the geometry and the rest geometry for simulation
    auto [geo, rest_geo] = object->geometries().create(mesh);

    // initialize the world using the scene
    world.init(scene);

    std::size_t total_frames = 10;

    REQUIRE_ALL_INFO(
        // main loop
        while(total_frames--) {
            world.advance();
            world.sync();
            world.retrieve();
        });
}
