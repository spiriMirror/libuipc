#pragma once
#include <animator/animator.h>
#include <affine_body/affine_body_dynamics.h>
#include <gradient_hessian_computer.h>
#include <line_search/line_searcher.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>

namespace uipc::backend::cuda
{
class AffineBodyConstraint;

class AffineBodyAnimator final : public Animator
{
  public:
    using Animator::Animator;

    class Impl;

    class AnimatedGeoInfo
    {
        // TODO:
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        span<const AnimatedGeoInfo> animated_geo_infos() const noexcept;

        span<const IndexT> anim_indices() const noexcept;

        SizeT anim_body_count() const noexcept;

      private:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, SizeT index, Float dt)
            : m_impl(impl)
            , m_index(index)
            , m_dt(dt)
        {
        }

        Float                                  dt() const noexcept;
        muda::CBufferView<Vector12>            qs() const noexcept;
        muda::CBufferView<ABDJacobiDyadicMass> masses() const noexcept;
        muda::CBufferView<IndexT>              is_fixed() const noexcept;

      protected:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
        Float m_dt    = 0.0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::BufferView<Float> energies() const noexcept;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::DoubletVectorView<Float, 12> gradients() const noexcept;
        muda::TripletMatrixView<Float, 12> hessians() const noexcept;
    };

    class ReportExtentInfo
    {
      public:
        void hessian_block_count(SizeT count) noexcept;
        void gradient_segment_count(SizeT count) noexcept;
        void energy_count(SizeT count) noexcept;

      private:
        friend class AffineBodyAnimator;
        SizeT m_hessian_block_count    = 0;
        SizeT m_gradient_segment_count = 0;
        SizeT m_energy_count           = 0;
    };

    class Impl
    {
      public:
        void init(backend::WorldVisitor& world);
        void step();

        Float dt = 0.0;

        AffineBodyDynamics* affine_body_dynamics = nullptr;
        SimSystemSlotCollection<AffineBodyConstraint> constraints;
        unordered_map<U64, SizeT>                     uid_to_constraint_index;

        vector<SizeT> anim_abd_geo_body_offsets;

        vector<AnimatedGeoInfo> animated_geo_infos;
        vector<IndexT>          anim_indices;

        vector<SizeT> constraint_geo_info_offsets;
        vector<SizeT> constraint_geo_info_counts;

        vector<SizeT> constraint_body_offsets;
        vector<SizeT> constraint_body_counts;

        // Constraints
        muda::DeviceVar<Float>    constraint_energy;
        muda::DeviceBuffer<Float> constraint_energies;
        vector<SizeT>             constraint_energy_offsets;
        vector<SizeT>             constraint_energy_counts;

        muda::DeviceDoubletVector<Float, 12> constraint_gradient;
        vector<SizeT>                        constraint_gradient_offsets;
        vector<SizeT>                        constraint_gradient_counts;

        muda::DeviceTripletMatrix<Float, 12> constraint_hessian;
        vector<SizeT>                        constraint_hessian_offsets;
        vector<SizeT>                        constraint_hessian_counts;
    };

  private:
    friend class AffineBodyConstraint;
    void add_constraint(AffineBodyConstraint* constraint);  // only be called by AffinieElementConstraint

    friend class ABDLineSearchReporter;
    Float compute_energy(LineSearcher::EnergyInfo& info);  // only be called by ABDLineSearchReporter

    friend class ABDGradientHessianComputer;
    void compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info);  // only be called by ABDGradientHessianComputer

    friend class ABDLinearSubsystem;
    class ExtentInfo
    {
      public:
        SizeT hessian_block_count;
    };
    void report_extent(ExtentInfo& info);  // only be called by ABDLinearSubsystem
    class AssembleInfo
    {
      public:
        muda::DoubletVectorView<Float, 12> gradients;
        muda::TripletMatrixView<Float, 12> hessians;
    };
    void assemble(AssembleInfo& info);  // only be called by ABDLinearSubsystem

    Impl m_impl;

    virtual void do_init() override;
    virtual void do_step() override;
    virtual void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda
