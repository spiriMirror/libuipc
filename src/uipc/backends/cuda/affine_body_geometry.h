#pragma once
#include <uipc/backends/cuda/sim_system.h>
#include <uipc/backends/cuda/sim_engine.h>

namespace uipc::backend::cuda
{
class AffineBodyGeometry : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    virtual void build() noexcept override;
};
}  // namespace uipc::backend::cuda
