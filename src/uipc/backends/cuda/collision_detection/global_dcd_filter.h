#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class SimplexDCDFilter;

class GlobalDCDFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class Impl
    {
      public:
        SimSystemSlot<SimplexDCDFilter> simplex_dcd_filter;
    };

    void add_filter(SimplexDCDFilter* filter);

    SimplexDCDFilter* simplex_filter() const;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;

    void detect();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda