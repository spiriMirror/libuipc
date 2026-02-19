#include <affine_body/affine_body_kinetic.h>
#include <finite_element/finite_element_diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void AffineBodyKinetic::do_build()
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
    
    BuildInfo this_info;
    do_build(this_info);
    
    m_impl.affine_body_dynamics->add_kinetic(this);

}

void AffineBodyKinetic::compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl, &info};
    do_compute_energy(this_info);
}

void AffineBodyKinetic::compute_gradient_hessian(ABDLinearSubsystem::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{&m_impl, &info};
    do_compute_gradient_hessian(this_info);
}
}  // namespace uipc::backend::cuda
