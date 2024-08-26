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

        /****************************************************
        *                   Broad Phase
        ****************************************************/
        
        muda::DeviceBuffer<AABB> codim_point_aabbs;
        muda::DeviceBuffer<AABB> point_aabbs;
        muda::DeviceBuffer<AABB> edge_aabbs;
        muda::DeviceBuffer<AABB> triangle_aabbs;

        AtomicCountingLBVH         lbvh_PP;
        muda::BufferView<Vector2i> candidate_PP_pairs;
        AtomicCountingLBVH         lbvh_PE;
        muda::BufferView<Vector2i> candidate_PE_pairs;
        AtomicCountingLBVH         lbvh_PT;
        muda::BufferView<Vector2i> candidate_PT_pairs;
        AtomicCountingLBVH         lbvh_EE;
        muda::BufferView<Vector2i> candidate_EE_pairs;


        /****************************************************
        *                   Narrow Phase
        ****************************************************/
        
        muda::DeviceBuffer<IndexT> PT_active_flags;
        muda::DeviceBuffer<IndexT> PT_active_offsets;
        muda::DeviceBuffer<IndexT> EE_active_flags;
        muda::DeviceBuffer<IndexT> EE_active_offsets;
        muda::DeviceBuffer<IndexT> PE_active_flags;
        muda::DeviceBuffer<IndexT> PE_active_offsets;
        muda::DeviceBuffer<IndexT> PP_active_flags;
        muda::DeviceBuffer<IndexT> PP_active_offsets;

        muda::DeviceBuffer<Vector4i> PTs;
        muda::DeviceBuffer<Vector4i> EEs;
        muda::DeviceBuffer<Vector3i> PEs;
        muda::DeviceBuffer<Vector2i> PPs;


        /****************************************************
        *                   CCD TOI
        ****************************************************/

        muda::DeviceBuffer<Float> tois;  // PP, PE, PT, EE
    };

  private:
    Impl m_impl;

    // Inherited via SimplexTrajectoryFilter
    virtual void do_detect(DetectInfo& info) override final;
    virtual void do_filter_active(FilterActiveInfo& info) override final;
    virtual void do_filter_toi(FilterTOIInfo& info) override final;
};
}  // namespace uipc::backend::cuda