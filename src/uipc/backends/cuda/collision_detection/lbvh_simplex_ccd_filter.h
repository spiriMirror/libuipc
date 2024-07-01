#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/atomic_counting_lbvh.h>
#include <collision_detection/simplex_ccd_filter.h>

namespace uipc::backend::cuda
{
class LBVHSimplexCCDFilter : public SimplexCCDFilter
{
  public:
    using SimplexCCDFilter::SimplexCCDFilter;

    class Impl
    {
      public:
        void broadphase_ccd(SimplexCCDFilter::FilterInfo& info);
        void filter_toi(SimplexCCDFilter::FilterInfo& info);

        muda::DeviceBuffer<AABB>     point_aabbs;
        muda::DeviceBuffer<AABB>     triangle_aabbs;
        AtomicCountingLBVH           lbvh_PT;
        muda::DeviceBuffer<Vector4i> candidate_PTs;

        muda::DeviceBuffer<AABB>     edge_aabbs;
        AtomicCountingLBVH           lbvh_EE;
        muda::DeviceBuffer<Vector4i> candidate_EEs;

        muda::DeviceBuffer<Float> tois;
    };

  protected:
    virtual void do_filter_toi(SimplexCCDFilter::FilterInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
