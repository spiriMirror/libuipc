#pragma once
#include <sim_system.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
class AffineBodyGeometry : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    virtual void build() noexcept override;
};
}  // namespace uipc::backend::cuda
