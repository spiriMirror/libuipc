#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitutions/affine_body.h>
#include <filesystem>

int main()
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;
    using namespace uipc::engine;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    std::string output_dir =
        fmt::format("{}/hello_affine_body/", AssetDir::output_path());
    fs::exists(output_dir) || fs::create_directories(output_dir);


    UIPCEngine engine{"cuda"};
    World      world{engine};

    Scene scene;
    {
        // create constitution and contact model
        auto& abd = scene.constitution_tabular().create<AffineBodyConstitution>();
        auto default_contact = scene.contact_tabular().default_element();

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.1);
        SimplicialComplexIO io{pre_transform};

        //{
        //    // just for test, add a useless object which won't be simulated
        //    auto useless_mesh = io.read(fmt::format("{}tet.msh", tetmesh_dir));
        //    label_surface(useless_mesh);
        //    label_triangle_orient(useless_mesh);

        //    auto useless_object = scene.objects().create("useless");
        //    {
        //        useless_object->geometries().create(useless_mesh);
        //    }
        //}

        // create geometry
        // auto mesh = io.read(fmt::format("{}cube.msh", tetmesh_dir));

        vector<Vector3> Vs = {Vector3{0, 1, 0},
                              Vector3{0, 0, 1},
                              Vector3{-std::sqrt(3) / 2, 0, -0.5},
                              Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs.begin(), Vs.end(), Vs.begin(), [&](auto& v) { return v * 0.2; });

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        auto mesh = tetmesh(Vs, Ts);
        label_surface(mesh);
        label_triangle_orient(mesh);
        auto mesh2 = mesh;

        {  // setup mesh
            {
                mesh.instances().resize(2);
                // apply constitution and contact model to the geometry
                abd.apply_to(mesh, 100.0_MPa);
                default_contact.apply_to(mesh);

                auto trans_view = view(mesh.transforms());
                auto is_fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
                auto is_fixed_view = view(*is_fixed);

                {
                    Transform t = Transform::Identity();
                    t.translation() = Vector3::UnitY() * 0.13 + Vector3::UnitY() * 0.5;

                    trans_view[0]    = t.matrix();
                    is_fixed_view[0] = 0;
                }

                {
                    Transform t = Transform::Identity();
                    t.translation() = Vector3::UnitY() * -0.13 + Vector3::UnitY() * 0.5;

                    trans_view[1] = t.matrix();
                    // fix the second cube
                    is_fixed_view[1] = 1;
                }
            }


            {
                mesh2.instances().resize(1);
                abd.apply_to(mesh2, 20.0_MPa);
                default_contact.apply_to(mesh2);

                auto trans_view = view(mesh2.transforms());

                {
                    Transform t = Transform::Identity();
                    t.translation() = Vector3::UnitY() * 0.5 + Vector3::UnitX() * 0.5;
                    trans_view[0] = t.matrix();
                }
            }
        }

        // create object
        auto object = scene.objects().create("cubes");
        {
            object->geometries().create(mesh);
            // object->geometries().create(mesh2);
        }
    }

    world.init(scene);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_dir, 0));

    for(int i = 1; i < 30; i++)
    {
        world.advance();
        world.sync();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", output_dir, i));
    }
}