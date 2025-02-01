#include <app/test_common.h>
#include <app/asset_dir.h>

#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <fstream>

void test_engine(std::string_view name)
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    auto this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{name, this_output_path};
    World  world{engine};

    Scene scene;

    AffineBodyConstitution abd;
    scene.constitution_tabular().insert(abd);

    auto& contact_tabular = scene.contact_tabular();
    auto& default_contact = contact_tabular.default_element();


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

    std::string output_path = fmt::format("{}/{}", this_output_path, name);

    fs::exists(output_path) || fs::create_directories(output_path);

    {
        std::ofstream f(fmt::format("{}/engine.json", output_path, name));
        f << engine.to_json().dump(4);
    }

    std::size_t total_frames = 1;

    // main loop
    while(world.frame() < total_frames)
    {
        world.advance();
        world.retrieve();
    }
}


TEST_CASE("engine", "[world]")
{
    test_engine("none");
}
