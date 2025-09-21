#include <dytopo_effect_system/dytopo_effect_reporter.h>

namespace uipc::backend::cuda
{
void DyTopoEffectReporter::do_build()
{
    auto& manager = require<GlobalDyTopoEffectManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}


void DyTopoEffectReporter::init()
{
    InitInfo info;
    do_init(info);
}

void DyTopoEffectReporter::do_init(InitInfo&) {}

void DyTopoEffectReporter::report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info)
{
    do_report_energy_extent(info);
}

void DyTopoEffectReporter::report_gradient_hessian_extent(GlobalDyTopoEffectManager::GradientHessianExtentInfo& info)
{
    do_report_gradient_hessian_extent(info);
}

void DyTopoEffectReporter::assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info)
{
    do_assemble(info);

    m_impl.gradients = info.gradients();
    m_impl.hessians  = info.hessians();
}

void DyTopoEffectReporter::compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info)
{
    do_compute_energy(info);

    m_impl.energies = info.energies();
}
}  // namespace uipc::backend::cuda
