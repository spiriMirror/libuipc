#include <contact_system/inter_primitive_constitution.h>

namespace uipc::backend::cuda
{
void InterPrimitiveConstitution::do_build()
{
    auto all_uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(all_uids.begin(), all_uids.end(), uid()))
    {
        throw SimSystemException(
            fmt::format("{} requires Constraint UID={}", name(), uid()));
    }

    auto& manager = require<InterPrimitiveConstitutionManager>();

    // let the subclass take care of its own build
    BuildInfo info;
    do_build(info);

    manager.add_constitution(this);
}

void InterPrimitiveConstitution::init(FilteredInfo& info)
{
    do_init(info);
}

void InterPrimitiveConstitution::report_energy_extent(EnergyExtentInfo& info)
{
    do_report_energy_extent(info);
}

void InterPrimitiveConstitution::compute_energy(ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void InterPrimitiveConstitution::report_gradient_hessian_extent(GradientHessianExtentInfo& info)
{
    do_report_gradient_hessian_extent(info);
}

void InterPrimitiveConstitution::compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}

U64 InterPrimitiveConstitution::uid() const noexcept
{
    return get_uid();
}
}  // namespace uipc::backend::cuda