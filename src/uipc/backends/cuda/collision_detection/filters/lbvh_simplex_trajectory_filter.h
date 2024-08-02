#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/atomic_counting_lbvh.h>
#include <collision_detection/simplex_trajectory_filter.h>

namespace uipc::backend::cuda
{
class LBVHSimplexTrajectoryFilter final : public SimplexTrajectoryFilter
{
  public:
    using SimplexTrajectoryFilter::SimplexTrajectoryFilter;

    class Impl
    {
      public:
        void detect(DetectInfo& info);
        void filter_active(FilterActiveInfo& info);
        void filter_toi(FilterTOIInfo& info);

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

        // for filter active
        muda::DeviceBuffer<Vector4i> PTs;
        muda::DeviceBuffer<Vector4i> EEs;
        muda::DeviceBuffer<Vector3i> PEs;
        muda::DeviceBuffer<Vector2i> PPs;

        muda::DeviceVar<IndexT> selected;

        muda::DeviceBuffer<Float> tois;
    };

  private:
    Impl m_impl;

    // Inherited via SimplexTrajectoryFilter
    virtual void do_detect(DetectInfo& info) override final;
    virtual void do_filter_active(FilterActiveInfo& info) override final;
    virtual void do_filter_toi(FilterTOIInfo& info) override final;
};
}  // namespace uipc::backend::cuda