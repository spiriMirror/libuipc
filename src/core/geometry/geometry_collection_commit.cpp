#include <uipc/geometry/geometry_collection_commit.h>

namespace uipc::geometry
{
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
    m_diff_geometries.reserve(dst.m_geometries.size());
    for(auto&& [id, dst_geo_slot] : dst.m_geometries)
    {
        auto src_geo_slot = src.find(id);
        if(src_geo_slot)
        {
            m_diff_geometries[id] = dst_geo_slot->geometry() - src_geo_slot->geometry();
        }
        else
        {
            m_diff_geometries[id] = GeometryCommit{dst_geo_slot->geometry()};
        }
    }
}
}  // namespace uipc::geometry