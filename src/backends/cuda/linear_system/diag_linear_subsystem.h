#pragma once
#include <sim_system.h>
#include <linear_system/global_linear_system.h>
namespace uipc::backend::cuda
{
/**
 * @brief A diag linear subsystem represents a submatrix of the global hessian and a subvector of the global gradient.
 */
class DiagLinearSubsystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info);

    virtual void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) = 0;
    virtual void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info) = 0;

    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) = 0;
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info)            = 0;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)  = 0;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) = 0;

  private:
    friend class GlobalLinearSystem;
    virtual void do_build() final override;

    void report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info);
    void receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info);

    void report_extent(GlobalLinearSystem::DiagExtentInfo& info);
    void assemble(GlobalLinearSystem::DiagInfo& info);
    void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
    void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);


    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
