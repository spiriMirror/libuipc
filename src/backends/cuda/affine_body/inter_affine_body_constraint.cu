#include <affine_body/inter_affine_body_constraint.h>

namespace uipc::backend::cuda
{
void InterAffineBodyConstraint::do_build()
{
    auto all_uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(all_uids.begin(), all_uids.end(), uid()))
    {
        throw SimSystemException(
            fmt::format("{} requires Constraint UID={}", name(), uid()));
    }

    auto& affine_body_animator = require<InterAffineBodyAnimator>();

    BuildInfo info;
    do_build(info);

    affine_body_animator.add_constraint(this);
}

U64 InterAffineBodyConstraint::uid() const noexcept
{
    return get_uid();
}

void InterAffineBodyConstraint::init(InterAffineBodyAnimator::FilteredInfo& info)
{
    do_init(info);
}

void InterAffineBodyConstraint::step(InterAffineBodyAnimator::FilteredInfo& info)
{
    do_step(info);
}

void InterAffineBodyConstraint::report_extent(InterAffineBodyAnimator::ReportExtentInfo& info)
{
    do_report_extent(info);
}

void InterAffineBodyConstraint::compute_energy(InterAffineBodyAnimator::EnergyInfo& info)
{
    do_compute_energy(info);
}

void InterAffineBodyConstraint::compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
