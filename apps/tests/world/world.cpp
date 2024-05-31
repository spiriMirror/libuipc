#include <app/test_common.h>
#include <app/asset_dir.h>

#include <uipc/world/scene.h>
#include <uipc/world/world.h>
#include <uipc/engine/engine.h>
#include <uipc/geometry.h>
#include <uipc/constitutions/affine_body.h>

using namespace uipc;
using namespace uipc::world;
using namespace uipc::geometry;
using namespace uipc::engine;

class NullEngine : public IEngine
{
    std::size_t m_frame = 0;

  public:
    void do_init(backend::WorldVisitor v) override {}
    void do_advance() override
    {
        m_frame++;
        fmt::print("[NullEngine] frame: {}\n", m_frame);
    }
    void do_sync() override {}
    void do_retrieve() override {}
};

TEST_CASE("simple_world", "[world]")
{
    NullEngine engine;
    World      world{engine};

    Scene scene;
    auto& contact_tabular = scene.contact_tabular();
    auto  default_contact = contact_tabular.default_element();

    auto& constitution_tabular = scene.constitution_tabular();
    auto& abd = constitution_tabular.create<constitution::AffineBodyConstitution>();


    auto object = scene.objects().create("cube");

    SimplicialComplexIO io;
    auto mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    abd.apply_to(mesh, 1e8);

    object->geometries().create(mesh);

    world.init(scene);

    std::size_t total_frames = 10;
    while(total_frames--)
    {
        world.advance();
        world.sync();
        world.retrieve();
    }
}
