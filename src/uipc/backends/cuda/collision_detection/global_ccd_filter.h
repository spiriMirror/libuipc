#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>

namespace uipc::backend::cuda
{
class SimplexCCDFilter;
class CCDFilter;
class GlobalCCDFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class FilterInfo
    {
      public:
        Float                alpha() const noexcept { return m_alpha; }
        muda::VarView<Float> toi() const noexcept { return m_toi; }

      private:
        friend class GlobalCCDFilter;
        Float                m_alpha = 0.0;
        muda::VarView<Float> m_toi;
    };

    class Impl
    {
      public:
        void  init();
        Float filter_toi(Float alpha);

        SimSystemSlotCollection<CCDFilter> filters;
        muda::DeviceBuffer<Float>      tois;
        vector<Float>                  h_tois;
    };

    void add_filter(CCDFilter* filter);

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    Float filter_toi(Float alpha);
    Impl  m_impl;
};
}  // namespace uipc::backend::cuda
