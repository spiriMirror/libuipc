#include <implicit_geometry/ground.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/implicit_geometry_uid_register.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(Ground);

//const Vector3& Ground::normal() const
//{
//    return m_impl.normal;
//}
//
//const Vector3& Ground::position() const
//{
//    return m_impl.position;
//}

void Ground::do_build()
{
    on_init_scene([this] { m_impl.init(world()); });
}

void Ground::Impl::init(WorldVisitor& world)
{
    _find_geometry(world);
    _build_geometry();
}

void Ground::Impl::_find_geometry(WorldVisitor& world)
{
    auto                              geo_slots = world.scene().geometries();
    IndexT                            found     = 0;
    P<geometry::ImplicitGeometrySlot> found_slot;

    for(auto slot : geo_slots)
    {
        geometry::Geometry* geo = &slot->geometry();
        if(geo->type() == builtin::ImplicitGeometry)
        {
            auto is_ground = geo->meta().find<IndexT>(builtin::is_ground);

            bool success = true;

            success &= (is_ground && is_ground->view()[0] == 1);
            auto implicit_geo = geo->as<ImplicitGeometry>();
            UIPC_ASSERT(implicit_geo, "dynamic_cast<ImplicitGeometry*> failed, why can it happen?");
            auto uid = implicit_geo->meta().find<U64>(builtin::implicit_geometry_uid);

            // UID=1 is half plane, according to the UIPC spec.
            success &= uid->view()[0] == 1ull;

            if(success)
            {
                if(found == 0)
                {
                    ground_geo = implicit_geo;
                }
                else
                {
                    spdlog::warn("Multiple ground geometries found, this one (slot id={}) is ignored.",
                                 slot->id());
                }
                found++;
            }
        }
    }

    if(found > 1)
    {
        spdlog::warn("Multiple ground geometries found, only the first one (slot id={}) will be used.",
                     found_slot->id());
    }
}

void Ground::Impl::_build_geometry()
{
    if(ground_geo)
    {
        auto P = ground_geo->instances().find<Vector3>("P");
        UIPC_ASSERT(P, "Ground geometry must have P attribute.");
        auto N = ground_geo->instances().find<Vector3>("N");
        UIPC_ASSERT(N, "Ground geometry must have N attribute.");

        position = P->view()[0];
        normal   = N->view()[0];
    }
}
}  // namespace uipc::backend::cuda
