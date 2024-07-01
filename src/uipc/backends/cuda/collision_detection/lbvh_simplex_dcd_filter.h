#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/atomic_counting_lbvh.h>
#include <collision_detection/simplex_dcd_filter.h>

namespace uipc::backend::cuda
{
class LBVHSimplexDCDFilter : public SimplexDCDFilter
{
  public:
    using SimplexDCDFilter::SimplexDCDFilter;

    class Impl
    {
      public:
        void detect(SimplexDCDFilter::FilterInfo& info);
        void classify(SimplexDCDFilter::FilterInfo& info);

        // broad phase

        muda::DeviceBuffer<AABB>     point_aabbs;
        muda::DeviceBuffer<AABB>     triangle_aabbs;
        AtomicCountingLBVH           lbvh_PT;
        muda::DeviceBuffer<Vector4i> candidate_PTs;

        muda::DeviceBuffer<AABB>     edge_aabbs;
        AtomicCountingLBVH           lbvh_EE;
        muda::DeviceBuffer<Vector4i> candidate_EEs;


        // narrow phase

        muda::DeviceBuffer<Vector4i> temp_PTs;
        muda::DeviceBuffer<Vector4i> temp_EEs;
        muda::DeviceBuffer<Vector3i> temp_PEs;
        muda::DeviceBuffer<Vector2i> temp_PPs;

        muda::DeviceBuffer<Vector4i> PTs;
        muda::DeviceBuffer<Vector4i> EEs;
        muda::DeviceBuffer<Vector3i> PEs;
        muda::DeviceBuffer<Vector2i> PPs;

        muda::DeviceVar<IndexT> selected;
    };

  protected:
    virtual void do_detect(SimplexDCDFilter::FilterInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
