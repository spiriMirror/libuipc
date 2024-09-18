#pragma once
#include <linear_system/diag_linear_subsystem.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_animator.h>
#include <affine_body/abd_contact_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <affine_body/matrix_converter.h>

namespace uipc::backend::cuda
{
class ABDLinearSubsystem final: public DiagLinearSubsystem
{
  public:
    using DiagLinearSubsystem::DiagLinearSubsystem;

    class Impl
    {
      public:
        void report_extent(GlobalLinearSystem::DiagExtentInfo& info);
        void assemble(GlobalLinearSystem::DiagInfo& info);
        void _assemble_gradient(GlobalLinearSystem::DiagInfo& info);
        void _assemble_hessian(GlobalLinearSystem::DiagInfo& info);
        void assemble_bodies();
        void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);

        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
        AffineBodyDynamics::Impl&         abd() noexcept
        {
            return affine_body_dynamics->m_impl;
        }
        SimSystemSlot<ABDContactReceiver> abd_contact_receiver;
        ABDContactReceiver::Impl&         contact() noexcept
        {
            return abd_contact_receiver->m_impl;
        }
        SimSystemSlot<AffineBodyVertexReporter> affine_body_vertex_reporter;
        SimSystemSlot<AffineBodyAnimator>       affine_body_animator;

        Float reserve_ratio = 1.5;

        bool reduce_hessian = false;

        muda::TripletMatrixView<Float, 12>   A_view;
        muda::DeviceTripletMatrix<Float, 12> triplet_A;

        ABDMatrixConverter                converter;
        muda::DeviceBCOOMatrix<Float, 12> bcoo_A;
    };

  protected:
    virtual void do_build(DiagLinearSubsystem::BuildInfo& info) override;
    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override;
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info) override;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) override;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
