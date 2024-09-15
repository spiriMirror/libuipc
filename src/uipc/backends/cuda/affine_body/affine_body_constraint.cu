#include <affine_body/affine_body_constraint.h>

namespace uipc::backend::cuda
{
U64 AffineBodyConstraint::uid() const noexcept
{
    return get_uid();
}

void AffineBodyConstraint::do_build() {}

void AffineBodyConstraint::init(AffineBodyAnimator::FilteredInfo& info)
{
    do_init(info);
}

void AffineBodyConstraint::step(AffineBodyAnimator::FilteredInfo& info)
{
    do_step(info);
}

void AffineBodyConstraint::report_extent(AffineBodyAnimator::ReportExtentInfo& info)
{
    do_report_extent(info);
}

void AffineBodyConstraint::compute_energy(AffineBodyAnimator::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void AffineBodyConstraint::compute_gradient_hessian(AffineBodyAnimator::ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
