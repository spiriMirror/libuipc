#include <cmath>
#include <filesystem>
#include <iostream>

#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

int main()
{
    using namespace uipc;
    using namespace uipc::constitution;
    using namespace uipc::core;
    using namespace uipc::geometry;

    const std::string workspace = "output/docs/fem_cpp";
    std::filesystem::create_directories(workspace);
    Engine engine{"cuda", workspace};
    World  world{engine};

    auto config                 = Scene::default_config();
    config["dt"]                = 0.01;
    config["gravity"]           = Vector3{0.0, -9.8, 0.0};
    config["contact"]["enable"] = false;
    Scene scene{config};

    const Float      s3         = std::sqrt(3.0) / 2.0;
    vector<Vector3>  vertices   = {Vector3{0.0, 1.0, 0.0},
                                   Vector3{0.0, 0.0, 1.0},
                                   Vector3{-s3, 0.0, -0.5},
                                   Vector3{s3, 0.0, -0.5}};
    vector<Vector4i> tetrahedra = {Vector4i{0, 1, 2, 3}};

    auto solid = tetmesh(vertices, tetrahedra);
    label_surface(solid);
    label_triangle_orient(solid);

    StableNeoHookean material;
    auto             moduli = ElasticModuli::youngs_poisson(50.0_kPa, 0.499);
    material.apply_to(solid, moduli, 1000.0);

    scene.objects().create("soft_tet")->geometries().create(solid);

    world.init(scene);
    if(!world.is_valid())
        return 1;
    while(world.frame() < 10)
    {
        world.advance();
        world.retrieve();
    }

    std::cout << "FEM scene reached frame " << world.frame() << '\n';
    return world.is_valid() ? 0 : 1;
}
