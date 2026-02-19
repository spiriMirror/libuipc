#include <affine_body/inter_affine_body_constitution.h>

namespace uipc::backend::cuda
{
void InterAffineBodyConstitution::do_build()
{
    auto all_uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(all_uids.begin(), all_uids.end(), uid()))
    {
        throw SimSystemException(
            fmt::format("{} requires Constraint UID={}", name(), uid()));
    }

    auto& manager = require<InterAffineBodyConstitutionManager>();

    // let the subclass take care of its own build
    BuildInfo info;
    do_build(info);

    manager.add_constitution(this);
}

void InterAffineBodyConstitution::init(FilteredInfo& info)
{
    do_init(info);
}

void InterAffineBodyConstitution::report_energy_extent(EnergyExtentInfo& info)
{
    do_report_energy_extent(info);
}

void InterAffineBodyConstitution::compute_energy(ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void InterAffineBodyConstitution::report_gradient_hessian_extent(GradientHessianExtentInfo& info)
{
    do_report_gradient_hessian_extent(info);

    UIPC_ASSERT(!(info.gradient_only() && info.m_hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name(),
                info.m_hessian_count);
}

void InterAffineBodyConstitution::compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}
U64 InterAffineBodyConstitution::uid() const noexcept
{
    return get_uid();
}
}  // namespace uipc::backend::cuda