#pragma once
#include <algorithm/matrix_converter.h>
#include <linear_system/diag_linear_subsystem.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_animator.h>
#include <finite_element/fem_contact_receiver.h>
#include <finite_element/finite_element_vertex_reporter.h>

namespace uipc::backend::cuda
{
class FEMLinearSubsystem : public DiagLinearSubsystem
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
        void _assemble_animation(GlobalLinearSystem::DiagInfo& info);
        void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);

        SimSystemSlot<FiniteElementMethod> finite_element_method;
        FiniteElementMethod::Impl&         fem() noexcept
        {
            return finite_element_method->m_impl;
        }
        SimSystemSlot<FEMContactReceiver> fem_contact_receiver;
        FEMContactReceiver::Impl&         contact() noexcept
        {
            return fem_contact_receiver->m_impl;
        }
        SimSystemSlot<FiniteElementVertexReporter> finite_element_vertex_reporter;
        SimSystemSlot<FiniteElementAnimator> finite_element_animator;
        FiniteElementAnimator::Impl&         animator() noexcept
        {
            return finite_element_animator->m_impl;
        }
        SizeT animator_hessian_offset = 0;
        SizeT animator_hessian_count  = 0;

        Float reserve_ratio = 1.5;

        MatrixConverter<Float, 3>           converter;
        muda::DeviceTripletMatrix<Float, 3> triplet_A;
        muda::DeviceBCOOMatrix<Float, 3>    bcoo_A;

        static constexpr SizeT H12x12_to_H3x3 = (12 * 12) / (3 * 3);
        static constexpr SizeT H9x9_to_H3x3   = (9 * 9) / (3 * 3);
        static constexpr SizeT H6x6_to_H3x3   = (6 * 6) / (3 * 3);
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
