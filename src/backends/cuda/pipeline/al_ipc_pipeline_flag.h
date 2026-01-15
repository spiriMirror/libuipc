#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
/*
* @brief AL-IPC Pipeline Flag
*
* This is a flag class, by `require<ALIPCPipelineFlag>()` you can easily shutdown your dependent
* systems if the AL-IPC pipeline is not enabled.
*/
class ALIPCPipelineFlag final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    void do_build() override;
};
}  // namespace uipc::backend::cuda
