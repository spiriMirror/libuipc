#pragma once
#include <affine_body/inter_affine_body_constraint.h>

namespace uipc::backend::cuda
{
class AffineBodyPrismaticJointExternalBodyForceConstraint final : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;

    static constexpr U64 UID = 667;

    class Impl
    {
      public:
        vector<Float>    h_forces;
        vector<Vector2i> h_body_ids;
        vector<Vector6>  h_rest_tangents;

        muda::DeviceBuffer<Float>    forces;
        muda::DeviceBuffer<Vector2i> body_ids;
        muda::DeviceBuffer<Vector6>  rest_tangents;
    };

    muda::CBufferView<Float>    forces() const noexcept;
    muda::CBufferView<Vector2i> body_ids() const noexcept;
    muda::CBufferView<Vector6>  rest_tangents() const noexcept;

  private:
    virtual void do_build(BuildInfo& info) override;
    virtual U64  get_uid() const noexcept override;
    virtual void do_init(InterAffineBodyAnimator::FilteredInfo& info) override;
    virtual void do_step(InterAffineBodyAnimator::FilteredInfo& info) override;
    virtual void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override;
    virtual void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
