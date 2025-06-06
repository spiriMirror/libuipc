#include <affine_body/affine_body_constraint.h>

namespace uipc::backend::cuda
{
void AffineBodyConstraint::do_build()
{
    auto all_uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(all_uids.begin(), all_uids.end(), uid()))
    {
        throw SimSystemException(
            fmt::format("{} requires Constraint UID={}", name(), uid()));
    }

    auto& affine_body_animator = require<AffineBodyAnimator>();

    BuildInfo info;
    do_build(info);

    affine_body_animator.add_constraint(this);
}

U64 AffineBodyConstraint::uid() const noexcept
{
    return get_uid();
}

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

void AffineBodyConstraint::compute_energy(AffineBodyAnimator::EnergyInfo& info)
{
    do_compute_energy(info);
}

void AffineBodyConstraint::compute_gradient_hessian(AffineBodyAnimator::GradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
