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

        // create geometry
        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.2);
        SimplicialComplexIO io{pre_transform};

        auto mesh = io.read(fmt::format("{}cube.msh", tetmesh_dir));

        label_surface(mesh);
        label_triangle_orient(mesh);

        {  // setup mesh

            // apply constitution and contact model to the geometry
            abd.apply_to(mesh, 100.0_MPa);
            default_contact.apply_to(mesh);

            mesh.instances().resize(2);
            auto trans_view = view(mesh.transforms());
            auto is_fixed   = mesh.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);

            {
                Transform t     = Transform::Identity();
                t.translation() = Vector3::UnitY() * 0.2;

                trans_view[0]    = t.matrix();
                is_fixed_view[0] = 0;
            }

            {
                Transform t     = Transform::Identity();
                t.translation() = Vector3::UnitY() * -0.2;

                trans_view[1] = t.matrix();
                // fix the second cube
                is_fixed_view[1] = 1;
            }
        }

        // create object
        auto object = scene.objects().create("cubes");
        {
            object->geometries().create(mesh);
        }
    }

    world.init(scene);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_dir, 0));

    for(int i = 1; i < 10; i++)
    {
        world.advance();
        world.sync();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", output_dir, i));
    }
}