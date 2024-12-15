#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_animator.h>

namespace uipc::backend::cuda
{
class AffineBodyConstraint : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                       = 0;
    virtual U64  get_uid() const noexcept                        = 0;
    virtual void do_init(AffineBodyAnimator::FilteredInfo& info) = 0;
    virtual void do_step(AffineBodyAnimator::FilteredInfo& info) = 0;
    virtual void do_report_extent(AffineBodyAnimator::ReportExtentInfo& info) = 0;
    virtual void do_compute_energy(AffineBodyAnimator::ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(AffineBodyAnimator::ComputeGradientHessianInfo& info) = 0;

  private:
    friend class AffineBodyAnimator;
    virtual void do_build() override final;

    void init(AffineBodyAnimator::FilteredInfo& info);
    void step(AffineBodyAnimator::FilteredInfo& info);
    void report_extent(AffineBodyAnimator::ReportExtentInfo& info);
    void compute_energy(AffineBodyAnimator::ComputeEnergyInfo& info);
    void compute_gradient_hessian(AffineBodyAnimator::ComputeGradientHessianInfo& info);

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
