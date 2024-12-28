#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>

TEST_CASE("8_abd_multi_contact_model", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["friction"]["enable"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        AffineBodyConstitution abd;
        scene.constitution_tabular().insert(abd);

        auto& contact_tabular = scene.contact_tabular();
        auto& default_contact = contact_tabular.default_element();
        auto& rubber_contact  = contact_tabular.create("rubber");

        contact_tabular.default_model(0.5, 1.0_GPa);
        contact_tabular.insert(default_contact, rubber_contact, 0.5, 100.0_MPa);
        contact_tabular.insert(rubber_contact, rubber_contact, 0.5, 10.0_MPa);

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.3);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto object = scene.objects().create("cubes");

        auto cube = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

        label_surface(cube);
        label_triangle_orient(cube);

        constexpr SizeT N = 2;
        cube.instances().resize(2);

        // wood cubes
        {
            SimplicialComplex wood_cube = cube;
            abd.apply_to(wood_cube, 100.0_MPa);
            default_contact.apply_to(wood_cube);

            auto trans_view = view(wood_cube.transforms());
            auto is_fixed = wood_cube.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            for(SizeT i = 0; i < N; i++)
            {
                Transform t      = Transform::Identity();
                t.translation()  = Vector3::UnitY() * 0.35 * i;
                trans_view[i]    = t.matrix();
                is_fixed_view[i] = 0;
            }

            is_fixed_view[0] = 1;

            object->geometries().create(wood_cube);
        }

        // rubber cubes
        {
            SimplicialComplex rubber_cube = cube;
            abd.apply_to(rubber_cube, 10.0_MPa);
            rubber_contact.apply_to(rubber_cube);

            auto trans_view = view(rubber_cube.transforms());
            auto is_fixed = rubber_cube.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            for(SizeT i = 0; i < N; i++)
            {
                Transform t      = Transform::Identity();
                t.translation()  = Vector3::UnitY() * 0.35 * (i + 2);
                trans_view[i]    = t.matrix();
                is_fixed_view[i] = 0;
            }

            object->geometries().create(rubber_cube);
        }
    }

    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 50; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}