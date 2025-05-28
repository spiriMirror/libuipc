#include <uipc/geometry/geometry_collection_commit.h>

namespace uipc::geometry
{
GeometryCollectionCommit::GeometryCollectionCommit(const GeometryCollectionCommit& other)
    : m_next_id{other.m_next_id}
{
    m_geometries.reserve(other.m_geometries.size());

    for(auto&& [id, commit] : other.m_geometries)
    {
        m_geometries[id] = uipc::make_shared<GeometryCommit>(*commit);
    }
}

GeometryCollectionCommit::GeometryCollectionCommit(const GeometryCollection& dst,
                                                   const GeometryCollection& src)
{
    UIPC_ASSERT(dst.m_pending_create.size() == 0,
                "GeometryCollectionCommit: The pending create size is not 0 (size={}), this is not expected.",
                dst.m_pending_create.size());

    UIPC_ASSERT(dst.m_pending_destroy.size() == 0,
                "GeometryCollectionCommit: The pending destroy size is not 0 (size={}), this is not expected.",
                dst.m_pending_destroy.size());

    UIPC_ASSERT(src.m_pending_create.size() == 0,
                "GeometryCollectionCommit: The pending create size is not 0 (size={}), this is not expected.",
                src.m_pending_create.size());

    UIPC_ASSERT(src.m_pending_destroy.size() == 0,
                "GeometryCollectionCommit: The pending destroy size is not 0 (size={}), this is not expected.",
                src.m_pending_destroy.size());


    m_next_id = dst.next_id();
    m_geometries.reserve(dst.m_geometries.size());
    for(auto&& [id, dst_geo_slot] : dst.m_geometries)
    {
        auto src_geo_slot = src.find(id);
        if(src_geo_slot)
        {
            m_geometries[id] = uipc::make_shared<GeometryCommit>(
                dst_geo_slot->geometry() - src_geo_slot->geometry());
        }
        else
        {
            m_geometries[id] =
                uipc::make_shared<GeometryCommit>(dst_geo_slot->geometry());
        }
    }
}
}  // namespace uipc::geometry