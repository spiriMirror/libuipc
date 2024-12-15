#pragma once
#include <collision_detection/vertex_half_plane_trajectory_filter.h>

namespace uipc::backend::cuda
{
class EasyVertexHalfPlaneTrajectoryFilter final : public VertexHalfPlaneTrajectoryFilter
{
  public:
    using VertexHalfPlaneTrajectoryFilter::VertexHalfPlaneTrajectoryFilter;

    class Impl
    {
      public:
        void filter_active(FilterActiveInfo& info);
        void filter_toi(FilterTOIInfo& info);

        muda::DeviceVar<IndexT> num_collisions;
        IndexT                  h_num_collisions;

        /**
         * @brief [Vertex-HalfPlane] pairs
         */
        muda::DeviceBuffer<Vector2i> PHs;

        Float reserve_ratio = 1.1f;

        muda::DeviceBuffer<Float> tois;
    };

  private:
    Impl m_impl;

    // Inherited via VertexHalfPlaneTrajectoryFilter

    virtual void do_detect(DetectInfo& info) override;
    virtual void do_filter_active(FilterActiveInfo& info) override;
    virtual void do_filter_toi(FilterTOIInfo& info) override;
};
}  // namespace uipc::backend::cuda
