#pragma once
#include <backends/common/sim_system.h>

namespace uipc::backend::none
{
class NoneSimSystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    void do_build() override;
};
}  // namespace uipc::backend::none
