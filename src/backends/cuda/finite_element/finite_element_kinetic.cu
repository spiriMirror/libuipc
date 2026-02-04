#include <finite_element/finite_element_kinetic.h>
#include <finite_element/finite_element_diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void FiniteElementKinetic::do_build()
{
    m_impl.finite_element_method = require<FiniteElementMethod>();

    BuildInfo this_info;
    do_build(this_info);

    m_impl.finite_element_method->add_kinetic(this);
}

void FiniteElementKinetic::compute_energy(FEMLineSearchReporter::ComputeEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl, &info};
    do_compute_energy(this_info);
}

void FiniteElementKinetic::compute_gradient_hessian(
    FEMLinearSubsystem::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{&m_impl, &info};
    do_compute_gradient_hessian(this_info);
}
}  // namespace uipc::backend::cuda
