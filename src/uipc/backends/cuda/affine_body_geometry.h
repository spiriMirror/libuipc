#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class AffineBodyGeometry : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    virtual void build() override;

    void init_affine_body_geometry();
};
}  // namespace uipc::backend::cuda
