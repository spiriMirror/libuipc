#include <finite_element/finite_element_constraint.h>

namespace uipc::backend::cuda
{
void FiniteElementConstraint::do_build()
{


    auto all_uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(all_uids.begin(), all_uids.end(), uid()))
    {
        throw SimSystemException(
            fmt::format("{} requires Constraint UID={}", name(), uid()));
    }

    auto& fem_animator = require<FiniteElementAnimator>();

    BuildInfo info;
    do_build(info);

    fem_animator.add_constraint(this);
}

void FiniteElementConstraint::init(FiniteElementAnimator::FilteredInfo& info)
{
    do_init(info);
}

void FiniteElementConstraint::step(FiniteElementAnimator::FilteredInfo& info)
{
    do_step(info);
}

void FiniteElementConstraint::report_extent(FiniteElementAnimator::ReportExtentInfo& info)
{
    do_report_extent(info);
}

void FiniteElementConstraint::compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void FiniteElementConstraint::compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}

U64 FiniteElementConstraint::uid() const noexcept
{
    return get_uid();
}
}  // namespace uipc::backend::cuda
