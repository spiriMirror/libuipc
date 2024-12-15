#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/type_define.h>
#include <uipc/common/span.h>
#include <uipc/common/smart_pointer.h>
#include <Eigen/Geometry>

namespace uipc::geometry
{
class UIPC_GEOMETRY_API Octree
{
  public:
    using AABB = Eigen::AlignedBox<Float, 3>;

    Octree();
    ~Octree();

    /**
     * @brief Build the octree from a list of AABBs
     * 
     * @param aabbs AABBs
     */
    void build(span<const AABB> aabbs);

    /**
     * @brief Clear the octree
     */
    void clear();

    /**
     * @brief Query the octree with a list of AABBs
     * 
     * @param aabbs AABBs
     * @param QF f:void(IndexT, IndexT), where the two indices are the indices of the two AABBs that intersect,
     * the first index is from the input list, and the second index is from the octree's AABBs.
     */
    void query(span<const AABB> aabbs, std::function<void(IndexT, IndexT)>&& QF) const;

    /**
     * @brief Detect the self-intersections of the octree
     * 
     * @param QF f:void(IndexT, IndexT), where the two indices are the indices of the two AABBs that intersect,
     * the two indices are from the octree's AABBs.
     */
    void detect(std::function<void(IndexT, IndexT)>&& QF) const;

  private:
    class Impl;
    U<Impl> m_impl;
};
}  // namespace uipc::geometry