#include <affine_body/affine_body_constitution.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
U64 AffineBodyConstitution::uid() const
{
    return get_uid();
}

void AffineBodyConstitution::do_build()
{
    auto& affine_body_dynamics = require<AffineBodyDynamics>();


    auto scene = world().scene();
    // Check if we have the Affine Body Constitution
    auto uids = scene.constitution_tabular().uids();
    if(!std::binary_search(uids.begin(), uids.end(), uid()))
    {
        throw SimSystemException(fmt::format("Requires Constitution UID={}", uid()));
    }

    BuildInfo info;
    do_build(info);

    affine_body_dynamics.add_constitution(this);
}

void AffineBodyConstitution::init(AffineBodyDynamics::FilteredInfo& info)
{
    return do_init(info);
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
