#include <contact_system/contact_reporter.h>

namespace uipc::backend::cuda
{
void ContactReporter::do_init(InitInfo&) {}

void ContactReporter::init()
{
    InitInfo info;
    do_init(info);
}

void ContactReporter::do_build()
{
    auto& manager = require<GlobalContactManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}

void ContactReporter::report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    do_report_energy_extent(info);
}

void ContactReporter::report_gradient_hessian_extent(GlobalContactManager::GradientHessianExtentInfo& info)
{
    do_report_gradient_hessian_extent(info);
}

void ContactReporter::assemble(GlobalContactManager::GradientHessianInfo& info)
{
    do_assemble(info);

    m_impl.gradients = info.gradients();
    m_impl.hessians  = info.hessians();
}

void ContactReporter::compute_energy(GlobalContactManager::EnergyInfo& info)
{
    do_compute_energy(info);

    m_impl.energies = info.energies();
}
}  // namespace uipc::backend::cuda
