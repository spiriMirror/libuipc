#include <affine_body/inter_affine_body_constraint.h>

namespace uipc::backend::cuda
{
span<const InterAffineBodyAnimator::AnimatedInterGeoInfo> InterAffineBodyConstraint::animated_inter_geo_info() const noexcept
{
    auto [offset, count] = m_animator->m_impl.constraint_geo_info_offsets_counts[m_index];
    return span{m_animator->m_impl.anim_geo_infos}.subspan(offset, count);
}

void InterAffineBodyConstraint::do_build()
{
    auto all_uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(all_uids.begin(), all_uids.end(), uid()))
    {
        throw SimSystemException(
            fmt::format("{} requires Constraint UID={}", name(), uid()));
    }

    m_animator = require<InterAffineBodyAnimator>();

    BuildInfo info;
    do_build(info);

    m_animator->add_constraint(this);
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
    info.check(name());

    UIPC_ASSERT(!(info.gradient_only() && info.m_hessian_block_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name(),
                info.m_hessian_block_count);
}

void InterAffineBodyConstraint::compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void InterAffineBodyConstraint::compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
