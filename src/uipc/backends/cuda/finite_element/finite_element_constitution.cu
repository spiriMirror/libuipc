#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
U64 FiniteElementConstitution::constitution_uid() const
{
    return get_constitution_uid();
}
IndexT FiniteElementConstitution::dimension() const
{
    return get_dimension();
}
void FiniteElementConstitution::do_build()
{
    m_fem = &require<FiniteElementMethod>();
    m_fem->add_constitution(this);
}
void FiniteElementConstitution::compute_energy(FiniteElementMethod::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}
void FiniteElementConstitution::compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
