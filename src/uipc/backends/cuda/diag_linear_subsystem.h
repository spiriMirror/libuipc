#pragma once
#include <sim_system.h>
#include <global_linear_system.h>
namespace uipc::backend::cuda
{
/**
 * @brief A diag linear subsystem represents a submatrix of the global hessian and a subvector of the global gradient.
 */
class DiagLinearSubsystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    friend class GlobalLinearSystem;
    virtual void report_extent(GlobalLinearSystem::DiagExtentInfo& info)   = 0;
    virtual void assemble(GlobalLinearSystem::DiagInfo& info)              = 0;
    virtual void retrieve_solution(GlobalLinearSystem::SolutionInfo& info) = 0;
};
}  // namespace uipc::backend::cuda
