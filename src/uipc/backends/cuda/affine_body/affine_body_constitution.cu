#include <affine_body/affine_body_constitution.h>

namespace uipc::backend::cuda
{
U64 AffineBodyConstitution::constitution_uid() const
{
    return get_constitution_uid();
}

void AffineBodyConstitution::filter(AffineBodyDynamics::FilteredInfo& info)
{
    return do_filter(info);
}

void AffineBodyConstitution::compute_energy(AffineBodyDynamics::ComputeEnergyInfo& info)
{
    return do_compute_energy(info);
}
void AffineBodyConstitution::compute_gradient_hessian(AffineBodyDynamics::ComputeGradientHessianInfo& info)
{
    return do_compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
