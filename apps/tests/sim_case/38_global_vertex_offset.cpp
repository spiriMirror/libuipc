#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/arap.h>
#include <uipc/constitution/affine_body_constitution.h>

TEST_CASE("38_global_vertex_offset", "[abd_fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                      = Vector3{0, -9.8, 0};
    config["contact"]["enable"]            = false;
    config["line_search"]["max_iter"]      = 8;
    config["linear_system"]["tol_rate"]    = 1e-3;
    config["line_search"]["report_energy"] = true;
    test::Scene::dump_config(config, output_path);

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

    auto u                             = scene.objects().create("upper");
    auto [u_geo_slot, u_rest_geo_slot] = u->geometries().create(upper_mesh);

    auto g                             = ground(-1.2);
    auto [g_geo_slot, g_rest_geo_slot] = object->geometries().create(g);


    world.init(scene);
    REQUIRE(world.is_valid());

    auto l_gvo = l_geo_slot->geometry().meta().find<IndexT>(builtin::global_vertex_offset);
    auto u_gvo = u_geo_slot->geometry().meta().find<IndexT>(builtin::global_vertex_offset);
    auto g_gvo = g_geo_slot->geometry().meta().find<IndexT>(builtin::global_vertex_offset);

    REQUIRE(l_gvo);
    REQUIRE(u_gvo);
    REQUIRE(g_gvo);

    fmt::println("l_gvo: {}, u_gvo: {}, g_gvo: {}",
                 l_gvo->view()[0],
                 u_gvo->view()[0],
                 g_gvo->view()[0]);
}
