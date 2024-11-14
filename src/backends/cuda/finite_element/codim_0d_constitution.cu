#include <finite_element/codim_0d_constitution.h>

namespace uipc::backend::cuda
{
void Codim0DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim0DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void Codim0DConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto& c_info = constitution_info();
    info.energy_count(0);  // not need to create any elastic energy
    info.stencil_dim(dim() + 1);
}

IndexT Codim0DConstitution::get_dim() const noexcept
{
    return 0;
}

void Codim0DConstitution::do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info)
{
    // do nothing
}

void Codim0DConstitution::do_compute_gradient_hessian(FiniteElementConstitution::ComputeGradientHessianInfo& info)
{
    // do nothing
}
}  // namespace uipc::backend::cuda
