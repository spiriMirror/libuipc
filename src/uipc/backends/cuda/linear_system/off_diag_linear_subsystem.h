#pragma once
#include <sim_system.h>
#include <linear_system/diag_linear_subsystem.h>
namespace uipc::backend::cuda
{
/**
 * @brief An off diag linear subsystem represents a submatrix of the global hessian
 */
class OffDiagLinearSubsystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;


  protected:
    virtual void report_extent(GlobalLinearSystem::OffDiagExtentInfo& info) = 0;
    virtual void assemble(GlobalLinearSystem::OffDiagInfo&)                 = 0;

  private:
    friend class GlobalLinearSystem;
    DiagLinearSubsystem* m_l = nullptr;
    DiagLinearSubsystem* m_r = nullptr;
    void depend_on(DiagLinearSubsystem* L, DiagLinearSubsystem* R);
};
}  // namespace uipc::backend::cuda
