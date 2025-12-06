#pragma once
#include <affine_body/affine_body_constraint.h>

namespace uipc::backend::cuda
{
/**
 * @brief Constraint that reads external forces from geometry attributes
 *
 * This constraint reads "external_force" attributes from geometries
 * and stores them inside its local buffers.
 *
 */
class AffineBodyExternalBodyForceConstraint final : public AffineBodyConstraint
{
  public:
    using AffineBodyConstraint::AffineBodyConstraint;

    static constexpr U64 UID = 666;  // Same UID as AffineBodyExternalForce

    class Impl
    {
      public:
        // Host-side buffers for collecting data before copying to device
        vector<Vector12>             h_forces;
        vector<IndexT>               h_body_ids;
        muda::DeviceBuffer<Vector12> forces;
        muda::DeviceBuffer<IndexT>   body_ids;
        AffineBodyDynamics*          affine_body_dynamics = nullptr;

        void step(backend::WorldVisitor& world, AffineBodyAnimator::FilteredInfo& info);
    };

    muda::CBufferView<Vector12> forces() const noexcept;
    muda::CBufferView<IndexT>   body_ids() const noexcept;

  private:
    virtual void do_build(BuildInfo& info) override;
    virtual U64  get_uid() const noexcept override;
    virtual void do_init(AffineBodyAnimator::FilteredInfo& info) override;
    virtual void do_step(AffineBodyAnimator::FilteredInfo& info) override;
    virtual void do_report_extent(AffineBodyAnimator::ReportExtentInfo& info) override;
    virtual void do_compute_energy(AffineBodyAnimator::EnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(AffineBodyAnimator::GradientHessianInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
