#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/arap.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>
#include <uipc/constitution/soft_vertex_stitch.h>

TEST_CASE("39_soft_vertex_stitch", "[abd_fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config = Scene::default_config();

    config["gravity"]                      = Vector3{0, -9.8, 0};
    config["contact"]["enable"]            = true;
    config["line_search"]["max_iter"]      = 8;
    config["linear_system"]["tol_rate"]    = 1e-3;
    config["line_search"]["report_energy"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};

    // create constitution and contact model
    StableNeoHookean       snh;
    AffineBodyConstitution abd;

    scene.contact_tabular().default_model(0.5, 1.0_GPa);
    auto default_element = scene.contact_tabular().default_element();

    // create object
    auto object = scene.objects().create("cubes");

    auto lower_mesh = io.read(fmt::format("{}cube.msh", tetmesh_dir));

    label_surface(lower_mesh);
    label_triangle_orient(lower_mesh);

    SimplicialComplex upper_mesh = lower_mesh;

    auto parm = ElasticModuli::youngs_poisson(20.0_kPa, 0.49);
    snh.apply_to(lower_mesh, parm);
    abd.apply_to(upper_mesh, 1.0_MPa);

    auto l                             = scene.objects().create("lower");
    auto [l_geo_slot, l_rest_geo_slot] = l->geometries().create(lower_mesh);

    auto      u = scene.objects().create("upper");
    Transform t = Transform::Identity();
    t.translate(Vector3{0, 1.5, 0});
    view(upper_mesh.transforms())[0] = t.matrix();
    auto is_fixed      = upper_mesh.instances().find<IndexT>(builtin::is_fixed);
    view(*is_fixed)[0] = 1;

    auto [u_geo_slot, u_rest_geo_slot] = u->geometries().create(upper_mesh);

    auto g                             = ground(-1.2);
    auto [g_geo_slot, g_rest_geo_slot] = object->geometries().create(g);


    auto             stich_obj = scene.objects().create("stitch");
    SoftVertexStitch stitch;
    vector<Vector2i> Es = {Vector2i{0, 4}};
    auto stitch_geo     = stitch.create_geometry({l_geo_slot, u_geo_slot}, Es);
    stich_obj->geometries().create(stitch_geo);

    world.init(scene);
    REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}