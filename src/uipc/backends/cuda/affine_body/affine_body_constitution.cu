#include <affine_body/affine_body_constitution.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
U64 AffineBodyConstitution::constitution_uid() const
{
    return get_constitution_uid();
}

void AffineBodyConstitution::do_build()
{
    auto& affine_body_dynamics = require<AffineBodyDynamics>();

    BuildInfo info;
    do_build(info);

    affine_body_dynamics.add_constitution(this);
}

void AffineBodyConstitution::retrieve(AffineBodyDynamics::FilteredInfo& info)
{
    return retrieve(info);
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
