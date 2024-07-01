#pragma once
#include <sim_system.h>
#include <muda/buffer/device_buffer.h>

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
        SimplexDCDFilter* filter = nullptr;
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