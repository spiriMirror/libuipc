#include <implicit_geometry/half_plane.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/implicit_geometry_uid_collection.h>
#include <uipc/common/range.h>
#include <uipc/common/enumerate.h>
#include <algorithm/geometry_instance_flattener.h>
#include <global_geometry/global_vertex_manager.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(HalfPlane);

void HalfPlane::do_build()
{
    on_init_scene([this] { m_impl.init(world()); });

    auto& global_vertex_manager = require<GlobalVertexManager>();
}

void HalfPlane::Impl::init(WorldVisitor& world)
{
    _find_geometry(world);
    _build_geometry();
}

void HalfPlane::Impl::_find_geometry(WorldVisitor& world)
{
    auto                              geo_slots = world.scene().geometries();
    list<geometry::ImplicitGeometry*> geo_buffer;

    for(auto slot : geo_slots)
    {
        geometry::Geometry* geo = &slot->geometry();
        if(geo->type() == builtin::ImplicitGeometry)
        {
            geo_buffer.push_back(geo->as<geometry::ImplicitGeometry>());
        }
    }

    geos.resize(geo_buffer.size());
    std::ranges::move(geo_buffer, geos.begin());
}

void HalfPlane::Impl::_build_geometry()
{
    GeometryInstanceFlattener flattener(span<ImplicitGeometry*>{geos});
    size_t instance_count = flattener.compute_instance_count();

    h_normals.reserve(instance_count);
    h_positions.reserve(instance_count);

    auto I2G = flattener.compute_instance_to_gemetry();

    flattener.flatten(
        [&](geometry::ImplicitGeometry* geo)
        {
            return std::make_tuple(geo->instances().find<Vector3>("N")->view(),
                                   geo->instances().find<Vector3>("P")->view());
        },
        [&](const Vector3& normals, const Vector3& positions)
        {
            h_normals.push_back(normals);
            h_positions.push_back(positions);
        });

    vector<IndexT> geo_to_contact_id(geos.size(), 0);
    vector<IndexT> geo_to_contact_masks(geos.size(), 0);

    for(auto&& [i, g] : enumerate(geos))
    {
        auto cid = g->meta().find<IndexT>(builtin::contact_element_id);
        geo_to_contact_id[i] = cid ? cid->view()[0] : 0;
    }

    h_contact_ids.resize(instance_count, 0);

    for(auto&& i : range(instance_count))
    {
        auto G           = I2G[i];
        h_contact_ids[i] = geo_to_contact_id[G];
    }

    positions.resize(h_positions.size());
    normals.resize(h_normals.size());
    contact_ids.resize(h_contact_ids.size());

    positions.view().copy_from(h_positions.data());
    normals.view().copy_from(h_normals.data());
    contact_ids.view().copy_from(h_contact_ids.data());
}

muda::CBufferView<Vector3> HalfPlane::normals() const
{
    return m_impl.normals;
}

muda::CBufferView<Vector3> HalfPlane::positions() const
{
    return m_impl.positions;
}
muda::CBufferView<IndexT> HalfPlane::contact_ids() const
{
    return m_impl.contact_ids;
}
}  // namespace uipc::backend::cuda
