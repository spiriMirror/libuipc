#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/simplex_trajectory_filter.h>

namespace uipc::backend::cuda
{
class InfoStacklessBVHSimplexTrajectoryFilter final : public SimplexTrajectoryFilter
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

        cuda_tool::DeviceBuffer<AABB>   codim_point_aabbs;
        cuda_tool::DeviceBuffer<AABB>   point_aabbs;
        cuda_tool::DeviceBuffer<AABB>   edge_aabbs;
        cuda_tool::DeviceBuffer<AABB>   triangle_aabbs;
        cuda_tool::DeviceBuffer<IndexT> codim_point_bids;
        cuda_tool::DeviceBuffer<IndexT> point_bids;
        cuda_tool::DeviceBuffer<IndexT> edge_bids;
        cuda_tool::DeviceBuffer<IndexT> triangle_bids;
        cuda_tool::DeviceBuffer<IndexT> codim_point_cids;
        cuda_tool::DeviceBuffer<IndexT> point_cids;
        cuda_tool::DeviceBuffer<IndexT> edge_cids;
        cuda_tool::DeviceBuffer<IndexT> triangle_cids;

        using ThisBVH = InfoStacklessBVH;

        // CodimP count always less or equal to AllP count.
        ThisBVH              lbvh_CodimP;
        ThisBVH::QueryBuffer candidate_AllP_CodimP_pairs;

        // Used to detect CodimP-AllE, and AllE-AllE pairs.
        ThisBVH              lbvh_E;
        ThisBVH::QueryBuffer candidate_CodimP_AllE_pairs;
        ThisBVH::QueryBuffer candidate_AllE_AllE_pairs;

        // Used to detect AllP-AllT pairs.
        ThisBVH              lbvh_T;
        ThisBVH::QueryBuffer candidate_AllP_AllT_pairs;

        // Four query counts and four compacted PP/PE/PT/EE counts are each
        // downloaded in one contiguous transfer.
        cuda_tool::DeviceBuffer<IndexT> query_counts;
        cuda_tool::DeviceBuffer<IndexT> selected_counts;

        cuda_tool::DeviceBuffer<Vector4i> temp_PTs;
        cuda_tool::DeviceBuffer<Vector4i> temp_EEs;
        cuda_tool::DeviceBuffer<Vector3i> temp_PEs;
        cuda_tool::DeviceBuffer<Vector2i> temp_PPs;

        cuda_tool::DeviceBuffer<Vector4i> PTs;
        cuda_tool::DeviceBuffer<Vector4i> EEs;
        cuda_tool::DeviceBuffer<Vector3i> PEs;
        cuda_tool::DeviceBuffer<Vector2i> PPs;


        /****************************************************
        *                   CCD TOI
        ****************************************************/

        cuda_tool::DeviceBuffer<Float> tois;  // PP, PE, PT, EE
    };

    virtual cuda_tool::CBufferView<Vector2i> candidate_PTs() const noexcept override;
    virtual cuda_tool::CBufferView<Vector2i> candidate_EEs() const noexcept override;
    virtual cuda_tool::CBufferView<Float> toi_PTs() const noexcept override;
    virtual cuda_tool::CBufferView<Float> toi_EEs() const noexcept override;

  private:
    Impl m_impl;

    virtual void do_build(BuildInfo& info) override final;
    virtual void do_detect(DetectInfo& info) override final;
    virtual void do_filter_active(FilterActiveInfo& info) override final;
    virtual void do_filter_toi(FilterTOIInfo& info) override final;
};
}  // namespace uipc::backend::cuda
