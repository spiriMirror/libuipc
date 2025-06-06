#include <affine_body/inter_affine_body_constitution.h>

namespace uipc::backend::cuda
{
void InterAffineBodyConstitution::do_build()
{
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