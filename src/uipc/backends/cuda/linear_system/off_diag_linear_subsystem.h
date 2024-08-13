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

    class BuildInfo
    {
      public:
        void connect(DiagLinearSubsystem* l, DiagLinearSubsystem* r)
        {
            m_diag_l = l;
            m_diag_r = r;
        }

      private:
        friend class OffDiagLinearSubsystem;
        DiagLinearSubsystem* m_diag_l = nullptr;
        DiagLinearSubsystem* m_diag_r = nullptr;
    };

  protected:
    virtual void report_extent(GlobalLinearSystem::OffDiagExtentInfo& info) = 0;
    virtual void assemble(GlobalLinearSystem::OffDiagInfo&)                 = 0;
    virtual void do_build(BuildInfo& info)                                  = 0;

  private:
    friend class GlobalLinearSystem;
    DiagLinearSubsystem* m_l = nullptr;
    DiagLinearSubsystem* m_r = nullptr;
    virtual void         do_build() final override;
};
}  // namespace uipc::backend::cuda
