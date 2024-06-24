#pragma once
#include <sim_system.h>
#include <linear_system/global_linear_system.h>
namespace uipc::backend::cuda
{
class GlobalPreconditioner : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_assemble(GlobalLinearSystem::GlobalPreconditionerAssemblyInfo& info) = 0;
    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) = 0;

  private:
    friend class GlobalLinearSystem;
    void assemble(GlobalLinearSystem::GlobalPreconditionerAssemblyInfo& info);
    void apply(GlobalLinearSystem::ApplyPreconditionerInfo& info);
};
}  // namespace uipc::backend::cuda
