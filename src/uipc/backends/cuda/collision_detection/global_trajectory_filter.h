#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>

namespace uipc::backend::cuda
{
class TrajectoryFilter;

class GlobalTrajectoryFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class FilterTOIInfo
    {
      public:
        Float                alpha() const noexcept { return m_alpha; }
        muda::VarView<Float> toi() const noexcept { return m_toi; }

      private:
        friend class GlobalTrajectoryFilter;
        Float                m_alpha = 0.0;
        muda::VarView<Float> m_toi;
    };

    class DetectInfo
    {
      public:
        Float alpha() const noexcept { return m_alpha; }

      private:
        friend class GlobalTrajectoryFilter;
        Float m_alpha = 0.0;
    };

    class FilterActiveInfo
    {
      public:
    };

    class RecordFrictionCandidatesInfo
    {
      public:
    };

    class Impl
    {
      public:
        void  init();
        Float filter_toi(Float alpha);

        SimSystemSlotCollection<TrajectoryFilter> filters;
        bool                                      friction_enabled = false;

        muda::DeviceBuffer<Float> tois;
        vector<Float>             h_tois;
    };

    template <std::derived_from<SimSystem> T>
    SimSystemSlot<T> find()
    {
        return m_impl.filters.find<T>();
    }

    void add_filter(TrajectoryFilter* filter);

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;

    void  detect(Float alpha);
    void  filter_active();
    Float filter_toi(Float alpha);

    void record_friction_candidates();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
