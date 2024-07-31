#include <uipc/uipc.h>
#include <uipc/constitutions/affine_body.h>

class NullEngine : public uipc::engine::IEngine
{
    std::size_t m_frame = 0;

  public:
    void   do_init(uipc::backend::WorldVisitor v) override;
    void   do_advance() override;
    void   do_sync() override {}
    void   do_retrieve() override {}
    size_t get_frame() const override { return m_frame; }
};


int main()
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;

    NullEngine engine;
    World      world{engine};
    Scene      scene;
    {
        // create constitution and contact model
        auto& abd = scene.constitution_tabular().create<AffineBodyConstitution>();
        auto default_contact = scene.contact_tabular().default_element();

        // create geometry
        std::vector       Vs  = {Vector3{0.0, 0.0, 0.0},
                                 Vector3{1.0, 0.0, 0.0},
                                 Vector3{0.0, 1.0, 0.0},
                                 Vector3{0.0, 0.0, 1.0}};
        std::vector       Ts  = {Vector4i{0, 1, 2, 3}};
        SimplicialComplex tet = tetmesh(Vs, Ts);

        // apply constitution and contact model to the geometry
        abd.apply_to(tet, 1e8);
        default_contact.apply_to(tet);

        // create object
        S<Object> object = scene.objects().create("tet");
        {
            object->geometries().create(tet);
        }

        // create a ground geometry
        ImplicitGeometry half_plane = ground(-1.0);

        S<Object> ground = scene.objects().create("ground");
        {
            ground->geometries().create(half_plane);
        }
    }
    world.init(scene);
    for(int i = 0; i < 10; i++)
    {
        world.advance();
        world.sync();
        world.retrieve();
    }
}

void NullEngine::do_init(uipc::backend::WorldVisitor v)
{
    using namespace uipc;

    auto print_geos = [&](span<S<geometry::GeometrySlot>> geos)
    {
        for(auto& geo : geos)
        {
            spdlog::info("[NullEngine] <{}>: {} ", geo->geometry().type(), geo->id());

            auto sc = dynamic_cast<uipc::geometry::SimplicialComplex*>(&geo->geometry());
            if(sc)
            {
                auto Vs = sc->positions().view();

                for(auto v : Vs)
                {
                    spdlog::info("[NullEngine] vertex: [{},{},{}]", v[0], v[1], v[2]);
                }

                auto Ts = sc->tetrahedra().topo().view();

                for(auto t : Ts)
                {
                    spdlog::info(
                        "[NullEngine] tetrahedra: [{},{},{},{}]", t[0], t[1], t[2], t[3]);
                }
            }

            auto ig = dynamic_cast<uipc::geometry::ImplicitGeometry*>(&geo->geometry());
            if(ig)
            {
                auto uid = ig->meta().find<uipc::U64>(uipc::builtin::implicit_geometry_uid);
                auto info = builtin::ImplicitGeometryUIDRegister::instance().find(
                    uid->view()[0]);
                spdlog::info("[NullEngine] Implicit Geometry: {} ({}) ", info.name, info.uid);

                auto N = ig->instances().find<uipc::Vector3>("N")->view()[0];
                auto P = ig->instances().find<uipc::Vector3>("P")->view()[0];

                spdlog::info("[NullEngine] N=[{},{},{}], P=[{},{},{}]",
                             N[0],
                             N[1],
                             N[2],
                             P[0],
                             P[1],
                             P[2]);
            }
        }
    };

    auto geos = v.scene().geometries();
    spdlog::info("[NullEngine] geometries:");
    print_geos(geos);
    spdlog::info("----------------------------------------");
    auto rest_geos = v.scene().rest_geometries();
    spdlog::info("[NullEngine] rest geometries:");
    print_geos(rest_geos);
    spdlog::info("----------------------------------------");
}

void NullEngine::do_advance()
{
    m_frame++;
    spdlog::info("[NullEngine] frame: {}", m_frame);
}
