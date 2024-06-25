#pragma once
#include <linear_system/diag_linear_subsystem.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/matrix_converter.h>
namespace uipc::backend::cuda
{
class ABDLinearSubsystem : public DiagLinearSubsystem
{
  public:
    using DiagLinearSubsystem::DiagLinearSubsystem;

    class Impl
    {
      public:
        AffineBodyDynamics*       m_abd = nullptr;
        AffineBodyDynamics::Impl& abd() { return m_abd->m_impl; }

        void report_extent(GlobalLinearSystem::DiagExtentInfo& info);
        void assemble(GlobalLinearSystem::DiagInfo& info);
        void _assemble_gradient(GlobalLinearSystem::DiagInfo& info);
        void _assemble_hessian(GlobalLinearSystem::DiagInfo& info);
        void _make_hessian_unique();
        void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);

        Float reserve_ratio       = 1.5;
        SizeT dof_count           = 0;
        SizeT hessian_block_count = 0;

        ABDMatrixConverter                   converter;
        muda::DeviceTripletMatrix<Float, 12> triplet_A;
        muda::DeviceBCOOMatrix<Float, 12>    bcoo_A;
    };

  protected:
    virtual void do_build() override;
    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override;
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info) override;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) override;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
