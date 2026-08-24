#include <filesystem>
#include <iostream>

#include <uipc/uipc.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/strain_limiting_baraff_witkin.h>

int main()
{
    using namespace uipc;
    using namespace uipc::constitution;
    using namespace uipc::core;
    using namespace uipc::geometry;

    const std::string workspace = "output/docs/cloth_cpp";
    std::filesystem::create_directories(workspace);
    Engine engine{"cuda", workspace};
    World  world{engine};

    auto config                = Scene::default_config();
    config["dt"]               = 0.01;
    config["gravity"]          = Vector3{0.0, -9.8, 0.0};
    config["contact"]["d_hat"] = 0.005;
    Scene scene{config};

    scene.contact_tabular().default_model(0.3, 100.0_MPa);
    auto default_contact = scene.contact_tabular().default_element();

    vector<Vector3>  vertices  = {Vector3{-0.5, 1.5, 0.0},
                                  Vector3{0.5, 1.5, 0.0},
                                  Vector3{-0.5, 0.5, 0.0},
                                  Vector3{0.5, 0.5, 0.0}};
    vector<Vector3i> triangles = {Vector3i{0, 2, 1}, Vector3i{1, 2, 3}};
    auto             cloth     = trimesh(vertices, triangles);
    label_surface(cloth);

    auto stretch = ElasticModuli2D::youngs_poisson(50.0_kPa, 0.40);
    auto shear   = ElasticModuli2D::youngs_poisson(0.5_kPa, 0.40);
    StrainLimitingBaraffWitkinShell membrane;
    membrane.apply_to(cloth, stretch, shear, 200.0, 0.001, 100.0);

    DiscreteShellBending bending;
    bending.apply_to(cloth, 50.0_kPa, 0.40);
    default_contact.apply_to(cloth);

    auto is_fixed = cloth.vertices().find<IndexT>(builtin::is_fixed);
    auto fixed    = view(*is_fixed);
    fixed[0]      = 1;
    fixed[1]      = 1;

    scene.objects().create("cloth")->geometries().create(cloth);

    auto floor = ground(0.0);
    default_contact.apply_to(floor);
    scene.objects().create("floor")->geometries().create(floor);

    world.init(scene);
    if(!world.is_valid())
        return 1;
    while(world.frame() < 10)
    {
        world.advance();
        world.retrieve();
    }

    std::cout << "cloth scene reached frame " << world.frame() << '\n';
    return world.is_valid() ? 0 : 1;
}
