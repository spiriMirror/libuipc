#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class GlobalCCDFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

  protected:
  private:
    friend class SimEngine;
    Float filter_toi(Float alpha);
};
}  // namespace uipc::backend::cuda
