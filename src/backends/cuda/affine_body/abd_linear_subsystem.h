#pragma once
#include <linear_system/diag_linear_subsystem.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_contact_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <affine_body/matrix_converter.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class ABDLinearSubsystemReporter;
class ABDLinearSubsystem final : public DiagLinearSubsystem
{
  public:
    using DiagLinearSubsystem::DiagLinearSubsystem;

    class ReportExtentInfo
    {
      public:
        // DoubletVector12 count
        void gradient_count(SizeT size);
        // TripletMatrix12x12 count
        void hessian_count(SizeT size);

      private:
        friend class ABDLinearSubsystem;
        SizeT m_gradient_count = 0;
        SizeT m_hessian_count  = 0;
    };

    class Impl;

    class AssembleInfo
    {
      public:
        AssembleInfo(Impl* impl, IndexT index) noexcept;
        muda::DoubletVectorView<Float, 12>     gradients() const;
        muda::TripletMatrixView<Float, 12, 12> hessians() const;

      private:
        friend class ABDLinearSubsystem;

        Impl*  m_impl  = nullptr;
        IndexT m_index = ~0ull;
    };

    class Impl
    {
      public:
        void init();
        void report_extent(GlobalLinearSystem::DiagExtentInfo& info);

        void report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info);
        void receive_init_dof_info(WorldVisitor& w, GlobalLinearSystem::InitDofInfo& info);

        void assemble(GlobalLinearSystem::DiagInfo& info);
        void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);

        SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;
        AffineBodyDynamics::Impl&               abd() const noexcept;
        SimSystemSlot<ABDContactReceiver>       abd_contact_receiver;
        ABDContactReceiver::Impl&               contact() const noexcept;
        SimSystemSlot<AffineBodyVertexReporter> affine_body_vertex_reporter;

        Float reserve_ratio = 1.5;

        SimSystemSlotCollection<ABDLinearSubsystemReporter> reporters;
        OffsetCountCollection<IndexT> reporter_gradient_offsets_counts;
        OffsetCountCollection<IndexT> reporter_hessian_offsets_counts;

        muda::DeviceTripletMatrix<Float, 12, 12> reporter_hessians;
        muda::DeviceDoubletVector<Float, 12>     reporter_gradients;

        Float dt = 0.0f;  // time step, used in assemble
    };

  private:
    virtual void do_build(DiagLinearSubsystem::BuildInfo& info) override;
    virtual void do_init(InitInfo& info) override;

    virtual void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override;
    virtual void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info) override;

    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override;
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info) override;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) override;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) override;

    friend class ABDLinearSubsystemReporter;
    void add_reporter(ABDLinearSubsystemReporter* reporter);  // only be called by ABDLinearSubsystemReporter

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
