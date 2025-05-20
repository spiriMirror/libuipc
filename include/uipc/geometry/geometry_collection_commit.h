#pragma once
#include <uipc/geometry/geometry_collection.h>
#include <uipc/geometry/geometry_commit.h>

namespace uipc::geometry
{
class UIPC_CORE_API GeometryCollectionCommit
{
    friend class GeometryFactory;

    friend UIPC_CORE_API GeometryCollectionCommit operator-(const GeometryCollection& dst,
                                                            const GeometryCollection& src);
    friend UIPC_CORE_API GeometryCollection& operator+=(GeometryCollection& dst,
                                                        const GeometryCollectionCommit& inc);

  public:
    GeometryCollectionCommit() = default;
    GeometryCollectionCommit(const GeometryCollection& dst, const GeometryCollection& src);

  private:
    static void update_from(GeometryCollection& base, const GeometryCollectionCommit& inc);

    IndexT m_next_id = 0;

    unordered_map<IndexT, GeometryCommit> m_diff_geometries;
};

UIPC_CORE_API GeometryCollectionCommit operator-(const GeometryCollection& dst,
                                                 const GeometryCollection& src);
UIPC_CORE_API GeometryCollection&      operator+=(GeometryCollection& dst,
                                             const GeometryCollectionCommit& inc);

}  // namespace uipc::geometry