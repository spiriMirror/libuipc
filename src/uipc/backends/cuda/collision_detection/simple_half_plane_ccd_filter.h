#pragma once
#include <collision_detection/half_plane_ccd_filter.h>

namespace uipc::backend::cuda
{
class SimpleHalfPlaneCCDFilter final : public HalfPlaneCCDFilter
{
  public:
    using HalfPlaneCCDFilter::HalfPlaneCCDFilter;

    class Impl
    {
      public:
        void filter_toi(FilterInfo& info);

        muda::DeviceBuffer<Float> tois;
    };

  private:
    virtual void do_filter_toi(FilterInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
