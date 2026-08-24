#include <cmath>
#include <filesystem>
#include <iostream>

#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

int main()
{
    using namespace uipc;
    using namespace uipc::constitution;
    using namespace uipc::core;
    using namespace uipc::geometry;

    const std::string workspace = "output/docs/rigid_body_cpp";
    std::filesystem::create_directories(workspace);
    Engine engine{"cuda", workspace};
    World  world{engine};

    auto config       = Scene::default_config();
    config["dt"]      = 0.01;
    config["gravity"] = Vector3{0.0, -9.8, 0.0};
    Scene scene{config};

    scene.contact_tabular().default_model(0.4, 1.0_GPa);
    auto default_contact = scene.contact_tabular().default_element();

    const Float     s3       = std::sqrt(3.0) / 2.0;
    vector<Vector3> vertices = {Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0},
                                Vector3{-s3, 0.0, -0.5},
                                Vector3{s3, 0.0, -0.5}};
    for(auto& p : vertices)
        p *= 0.3;
    vector<Vector4i> tetrahedra = {Vector4i{0, 1, 2, 3}};

    auto body = tetmesh(vertices, tetrahedra);
    label_surface(body);
    label_triangle_orient(body);

    AffineBodyConstitution abd;
    abd.apply_to(body, 100.0_MPa, 1000.0);
    default_contact.apply_to(body);

    Transform transform        = Transform::Identity();
    transform.translation()    = Vector3{0.0, 1.2, 0.0};
    view(body.transforms())[0] = transform.matrix();
    scene.objects().create("falling_body")->geometries().create(body);

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

    std::cout << "rigid-body scene reached frame " << world.frame() << '\n';
    return world.is_valid() ? 0 : 1;
}
