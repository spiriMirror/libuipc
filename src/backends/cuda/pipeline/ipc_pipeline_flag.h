#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
/*
* @brief IPC Pipeline Flag
*
* This is a flag class, by `require<IPCPipelineFlag>()` you can easily shutdown your dependent
* systems if the IPC pipeline is not enabled.
*/
class IPCPipelineFlag final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    void do_build() override;
};
}  // namespace uipc::backend::cuda
