#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <filesystem>
#include <fstream>

TEST_CASE("7_abd_tet_pile", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

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
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto& default_contact = scene.contact_tabular().default_element();

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.3);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 1, 0},
                               Vector3{0, 0, 1},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs.begin(), Vs.end(), Vs.begin(), [&](auto& v) { return v * 0.3; });

        auto mesh = tetmesh(Vs, Ts);

        label_surface(mesh);
        label_triangle_orient(mesh);

        constexpr SizeT N = 4;
        mesh.instances().resize(N);
        abd.apply_to(mesh, 100.0_MPa);
        default_contact.apply_to(mesh);

        auto trans_view    = view(mesh.transforms());
        auto is_fixed      = mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);


        for(SizeT i = 0; i < N; i++)
        {
            Transform t = Transform::Identity();

            if(i == 1)
            {
                t.translation()  = Vector3::UnitY() * 0.5 * i;
                is_fixed_view[i] = 0;
            }
            else
            {
                t.translation()  = Vector3::UnitY() * 0.5 * i;
                is_fixed_view[i] = 0;
            }
            trans_view[i] = t.matrix();
        }

        is_fixed_view[0] = 1;

        object->geometries().create(mesh);
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