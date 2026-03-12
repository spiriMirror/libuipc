#pragma once
#include <affine_body/inter_affine_body_constraint.h>

namespace uipc::backend::cuda
{
class AffineBodyRevoluteJointExternalBodyForceConstraint final : public InterAffineBodyConstraint
{
  public:
    using InterAffineBodyConstraint::InterAffineBodyConstraint;

    static constexpr U64 UID = 668;

    class Impl
    {
      public:
        vector<Float>     h_torques;
        vector<Vector2i>  h_body_ids;
        vector<Vector12>  h_rest_positions;
        vector<Float>     h_init_angles;
        vector<Float>     h_current_angles;

        muda::DeviceBuffer<Float>     torques;
        muda::DeviceBuffer<Vector2i>  body_ids;
        muda::DeviceBuffer<Vector12>  rest_positions;
        muda::DeviceBuffer<Float>     init_angles;
        muda::DeviceBuffer<Float>     current_angles;
    };

    muda::CBufferView<Float>     torques() const noexcept;
    muda::CBufferView<Vector2i>  body_ids() const noexcept;
    muda::CBufferView<Vector12>  rest_positions() const noexcept;
    muda::CBufferView<Float>     init_angles() const noexcept;
    muda::DeviceBuffer<Float>&   current_angles() noexcept;

  private:
    virtual void do_build(BuildInfo& info) override;
    virtual U64  get_uid() const noexcept override;
    virtual void do_init(InterAffineBodyAnimator::FilteredInfo& info) override;
    virtual void do_step(InterAffineBodyAnimator::FilteredInfo& info) override;
    virtual void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) override;
    virtual void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) override;

    void write_scene();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
