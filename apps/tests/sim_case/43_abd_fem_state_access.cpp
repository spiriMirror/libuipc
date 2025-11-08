#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <uipc/core/finite_element_state_accessor_feature.h>

TEST_CASE("43_abd_fem_state_access", "[abd]")
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

    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};

    // create constitution and contact model
    StableNeoHookean       snh;
    AffineBodyConstitution abd;

    // create object
    auto object    = scene.objects().create("cubes");
    auto base_mesh = io.read(fmt::format("{}cube.msh", tetmesh_dir));

    label_surface(base_mesh);
    label_triangle_orient(base_mesh);

    SimplicialComplex mesh_a = base_mesh;
    SimplicialComplex mesh_b = base_mesh;

    auto parm = ElasticModuli::youngs_poisson(20.0_kPa, 0.49);
    snh.apply_to(mesh_a, parm);

    abd.apply_to(mesh_b, 1.0_MPa);
    auto      trans_view = view(mesh_b.transforms());
    Transform t          = Transform::Identity();
    t.translate(Vector3{0.0, 1.0, 0.0});  // move a little bit
    trans_view[0] = t.matrix();

    object->geometries().create(mesh_a);
    object->geometries().create(mesh_b);

    auto g = ground(-1.2);
    object->geometries().create(g);


    world.init(scene);
    REQUIRE(world.is_valid());

    auto abd_accessor = world.features().find<AffineBodyStateAccessorFeature>();
    auto fem_accessor = world.features().find<FiniteElementStateAccessorFeature>();


    SimplicialComplex abd_state = abd_accessor->create_geometry();
    // no transform attribute by default
    REQUIRE(abd_state.instances().find<Matrix4x4>(builtin::transform) == nullptr);
    // need to get transform attribute, so create it here
    abd_state.instances().create<Matrix4x4>(builtin::transform);

    SimplicialComplex fem_state = fem_accessor->create_geometry();
    // no position attribute by default
    REQUIRE(fem_state.vertices().find<Vector3>(builtin::position) == nullptr);
    fem_state.vertices().create<Vector3>(builtin::position);  // need to get position attribute

    REQUIRE(fem_accessor->vertex_count() == mesh_a.vertices().size());
    REQUIRE(abd_accessor->body_count() == mesh_b.instances().size());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 100)
    {
        if(world.frame() == 50)
        {
            abd_accessor->copy_to(abd_state);
            fem_accessor->copy_to(fem_state);

            // set to initial position / transform
            auto pos_view      = view(fem_state.positions());
            auto init_pos_view = mesh_a.positions().view();
            std::ranges::copy(init_pos_view, pos_view.begin());

            auto trans_view      = view(abd_state.transforms());
            auto init_trans_view = mesh_b.transforms().view();
            std::ranges::copy(init_trans_view, trans_view.begin());

            // apply modified state back
            abd_accessor->copy_from(abd_state);
            fem_accessor->copy_from(fem_state);
        }

        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}