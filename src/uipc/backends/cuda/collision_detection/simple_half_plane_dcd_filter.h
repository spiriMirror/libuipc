#pragma once
#include <collision_detection/half_plane_dcd_filter.h>

namespace uipc::backend::cuda
{
class SimpleHalfPlaneDCDFilter final: public HalfPlaneDCDFilter
{
  public:
    using HalfPlaneDCDFilter::HalfPlaneDCDFilter;

    class Impl;

    class Impl
    {
      public:
        void detect(FilterInfo& info);

        muda::DeviceVar<IndexT> num_collisions;
        IndexT                  h_num_collisions;

        /**
         * @brief [Vertex-HalfPlane] pairs
         */
        muda::DeviceBuffer<Vector2i> PHs;

        Float reserve_ratio = 1.5f;
    };

    muda::CBufferView<Vector2i> PHs() const;

  private:
    virtual void do_detect(FilterInfo& info) override final;
    Impl         m_impl;
};
}  // namespace uipc::backend::cuda
