#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>

TEST_CASE("9_abd_multi_constitution", "[abd]")
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

        AffineBodyConstitution abd_ortho;
        scene.constitution_tabular().insert(abd_ortho);

        Json config    = AffineBodyConstitution::default_config();
        config["name"] = "ARAP";
        AffineBodyConstitution abd_arap{config};
        scene.constitution_tabular().insert(abd_arap);

        auto& contact_tabular = scene.contact_tabular();
        auto& default_contact = contact_tabular.default_element();
        contact_tabular.default_model(0.5, 1.0_GPa);

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.3);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto object = scene.objects().create("cubes");

        auto cube = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

        label_surface(cube);
        label_triangle_orient(cube);

        constexpr SizeT N = 4;
        cube.instances().resize(N);

        {
            SimplicialComplex arap_cube = cube;
            abd_arap.apply_to(arap_cube, 0.1_MPa);
            default_contact.apply_to(arap_cube);

            auto trans_view = view(arap_cube.transforms());
            auto is_fixed = arap_cube.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            for(SizeT i = 0; i < N; i++)
            {
                Transform t = Transform::Identity();
                t.translation() = Vector3::UnitY() * 0.35 * i + Vector3::UnitX() * 0.5;
                trans_view[i]    = t.matrix();
                is_fixed_view[i] = 0;
            }

            is_fixed_view[0] = 1;

            object->geometries().create(arap_cube);
        }

        {
            SimplicialComplex ortho_cube = cube;
            abd_ortho.apply_to(ortho_cube, 0.1_MPa);
            default_contact.apply_to(ortho_cube);

            auto trans_view = view(ortho_cube.transforms());
            auto is_fixed = ortho_cube.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            for(SizeT i = 0; i < N; i++)
            {
                Transform t      = Transform::Identity();
                t.translation()  = Vector3::UnitY() * 0.35 * i;
                trans_view[i]    = t.matrix();
                is_fixed_view[i] = 0;
            }

            is_fixed_view[0] = 1;

            object->geometries().create(ortho_cube);
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