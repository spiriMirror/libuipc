#include <cmath>
#include <filesystem>
#include <iostream>

#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>

int main()
{
    using namespace uipc;
    using namespace uipc::constitution;
    using namespace uipc::core;
    using namespace uipc::geometry;

    const std::string workspace = "output/docs/rigid_soft_cpp";
    std::filesystem::create_directories(workspace);
    Engine engine{"cuda", workspace};
    World  world{engine};

    auto config       = Scene::default_config();
    config["dt"]      = 0.01;
    config["gravity"] = Vector3{0.0, -9.8, 0.0};
    Scene scene{config};

    auto& table         = scene.contact_tabular();
    auto  rigid_contact = table.default_element();
    auto  soft_contact  = table.create("soft");
    table.default_model(0.3, 1.0_GPa);
    table.insert(rigid_contact, soft_contact, 0.5, 300.0_MPa);

    const Float     s3       = std::sqrt(3.0) / 2.0;
    vector<Vector3> vertices = {Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0},
                                Vector3{-s3, 0.0, -0.5},
                                Vector3{s3, 0.0, -0.5}};
    for(auto& p : vertices)
        p *= 0.3;
    vector<Vector4i> tetrahedra = {Vector4i{0, 1, 2, 3}};
    auto             base       = tetmesh(vertices, tetrahedra);
    label_surface(base);
    label_triangle_orient(base);

    auto soft = base;
    for(auto& p : view(soft.positions()))
        p += Vector3{0.0, 0.25, 0.0};
    StableNeoHookean fem;
    auto             moduli = ElasticModuli::youngs_poisson(100.0_kPa, 0.45);
    fem.apply_to(soft, moduli, 1000.0);
    soft_contact.apply_to(soft);
    scene.objects().create("soft_body")->geometries().create(soft);

    auto                   rigid = base;
    AffineBodyConstitution abd;
    abd.apply_to(rigid, 100.0_MPa, 1000.0);
    rigid_contact.apply_to(rigid);
    Transform transform         = Transform::Identity();
    transform.translation()     = Vector3{0.0, 1.0, 0.0};
    view(rigid.transforms())[0] = transform.matrix();
    scene.objects().create("rigid_body")->geometries().create(rigid);

    auto floor = ground(0.0);
    rigid_contact.apply_to(floor);
    scene.objects().create("floor")->geometries().create(floor);

    world.init(scene);
    if(!world.is_valid())
        return 1;
    while(world.frame() < 15)
    {
        world.advance();
        world.retrieve();
    }

    std::cout << "rigid-soft scene reached frame " << world.frame() << '\n';
    return world.is_valid() ? 0 : 1;
}
