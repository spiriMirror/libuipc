#include <finite_element/codim_0d_constitution.h>

namespace uipc::backend::cuda
{
void Codim0DConstitution::retrieve(FiniteElementMethod::Codim0DFilteredInfo& info)
{
    UIPC_ASSERT(false, "Not implemented");
}
void Codim0DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim0DConstitution::BuildInfo this_info;
    do_build(this_info);
}

IndexT Codim0DConstitution::get_dimension() const
{
    return 0;
}

void Codim0DConstitution::do_compute_energy(FiniteElementMethod::ComputeEnergyInfo& info)
{
    // do nothing
}

void Codim0DConstitution::do_compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info)
{
    // do nothing
}
}  // namespace uipc::backend::cuda
