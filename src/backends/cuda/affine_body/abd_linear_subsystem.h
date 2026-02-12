#pragma once
#include <linear_system/diag_linear_subsystem.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_dytopo_effect_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class ABDLinearSubsystemReporter;
class ABDLinearSubsystem final : public DiagLinearSubsystem
{
  public:
    using DiagLinearSubsystem::DiagLinearSubsystem;

    muda::CBufferView<Matrix12x12> diag_hessian() const noexcept
    {
        return m_impl.diag_hessian.view();
    }

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(bool                          gradient_only,
                                   muda::BufferView<Vector12>    gradient,
                                   muda::BufferView<Matrix12x12> hessians,
                                   Float                         dt) noexcept
            : m_gradient_only(gradient_only)
            , m_gradients(gradient)
            , m_hessians(hessians)
            , m_dt(dt)
        {
        }

        auto gradient_only() const noexcept { return m_gradient_only; }
        auto hessians() const noexcept { return m_hessians; }
        auto gradients() const noexcept { return m_gradients; }
        auto dt() const noexcept { return m_dt; }

      private:
        bool                          m_gradient_only = false;
        muda::BufferView<Matrix12x12> m_hessians;
        muda::BufferView<Vector12>    m_gradients;
        Float                         m_dt = 0.0;
    };

    class ReportExtentInfo
    {
      public:
        // DoubletVector12 count
        void gradient_count(SizeT size);
        // TripletMatrix12x12 count
        void hessian_count(SizeT size);
        bool gradient_only() const noexcept
        {
            m_gradient_only_checked = true;
            return m_gradient_only;
        }
        void check(std::string_view name) const;

      private:
        friend class ABDLinearSubsystem;
        friend class ABDLinearSubsystemReporter;
        SizeT m_gradient_count = 0;
        SizeT m_hessian_count  = 0;
        bool  m_gradient_only  = false;
        mutable bool m_gradient_only_checked = false;
    };

    class Impl;

    class AssembleInfo
    {
      public:
        AssembleInfo(Impl* impl, IndexT index, bool gradient_only) noexcept;
        muda::DoubletVectorView<Float, 12>     gradients() const;
        muda::TripletMatrixView<Float, 12, 12> hessians() const;
        bool                                   gradient_only() const noexcept;

      private:
        friend class ABDLinearSubsystem;

        Impl*  m_impl          = nullptr;
        IndexT m_index         = ~0;
        bool   m_gradient_only = false;
    };

    class Impl
    {
      public:
        void init();
        void report_extent(GlobalLinearSystem::DiagExtentInfo& info);

        void report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info);
        void receive_init_dof_info(WorldVisitor& w, GlobalLinearSystem::InitDofInfo& info);

        void assemble(GlobalLinearSystem::DiagInfo& info);
        void _assemble_kinetic_shape(IndexT& offset, GlobalLinearSystem::DiagInfo& info);
        void _assemble_reporters(IndexT& offset, GlobalLinearSystem::DiagInfo& info);
        void _assemble_dytopo_effect(IndexT& offset, GlobalLinearSystem::DiagInfo& info);

        void accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void retrieve_solution(GlobalLinearSystem::SolutionInfo& info);

        SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;
        AffineBodyDynamics::Impl&               abd() const noexcept;
        SimSystemSlot<ABDDyTopoEffectReceiver>  dytopo_effect_receiver;
        SimSystemSlot<AffineBodyVertexReporter> affine_body_vertex_reporter;

        Float reserve_ratio = 1.5;

        SimSystemSlotCollection<ABDLinearSubsystemReporter> reporters;
        OffsetCountCollection<IndexT> reporter_gradient_offsets_counts;
        OffsetCountCollection<IndexT> reporter_hessian_offsets_counts;

        muda::DeviceTripletMatrix<Float, 12, 12> reporter_hessians;
        muda::DeviceDoubletVector<Float, 12>     reporter_gradients;

        // intermediate gradient/hessian buffers for kinetic/shape
        muda::DeviceBuffer<Matrix12x12> body_id_to_shape_hessian;
        muda::DeviceBuffer<Vector12>    body_id_to_shape_gradient;
        muda::DeviceBuffer<Matrix12x12> body_id_to_kinetic_hessian;
        muda::DeviceBuffer<Vector12>    body_id_to_kinetic_gradient;

        // diag hessian for preconditioner
        muda::DeviceBuffer<Matrix12x12> diag_hessian;

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

    virtual U64 get_uid() const noexcept override;

    friend class ABDLinearSubsystemReporter;
    void add_reporter(ABDLinearSubsystemReporter* reporter);  // only be called by ABDLinearSubsystemReporter

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
