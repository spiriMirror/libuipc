#include <affine_body/constitutions/abd_ortho_potential.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDOrthoPotential);

void ABDOrthoPotential::build()
{
    m_impl.affine_body_geometry = find<AffineBodyDynamics>();

    // ConstitutionRegister the action to filter the affine body geometry
    m_impl.affine_body_geometry->on_update(
        ConstitutionUID,  // By libuipc specification
        [this](const AffineBodyDynamics::FilteredInfo& info)
        { m_impl.on_filter(info); },
        [this](const AffineBodyDynamics::ComputeEnergyInfo& info) {

        },
        [this](const AffineBodyDynamics::ComputeGradientHessianInfo& info) {

        });
}

void ABDOrthoPotential::Impl::on_filter(const AffineBodyDynamics::FilteredInfo& info)
{
    spdlog::info("Filtering ABDOrthoPotential");
}
}  // namespace uipc::backend::cuda
