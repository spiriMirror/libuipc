#include <finite_element/finite_element_energy_producer.h>

namespace uipc::backend::cuda
{
void FiniteElementEnergyProducer::do_build()
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();

    BuildInfo info;
    do_build(info);
}

void FiniteElementEnergyProducer::collect_extent_info()
{
    ReportExtentInfo info;
    do_report_extent(info);

    m_impl.energy_count   = info.m_energy_count;
    m_impl.gradient_count = info.m_gradient_count;
    m_impl.hessian_count  = info.m_hessian_count;
}

void FiniteElementEnergyProducer::compute_energy(LineSearcher::EnergyInfo& info)
{
    auto global_energies =
        m_impl.finite_element_method->m_impl.energy_producer_energies.view();
    ComputeEnergyInfo this_info{
        info.dt(), global_energies.subview(m_impl.energy_offset, m_impl.energy_count)};
    do_compute_energy(this_info);
}

void FiniteElementEnergyProducer::assemble_gradient_hessian(AssemblyInfo& info)
{
    auto global_gradient_view =
        m_impl.finite_element_method->m_impl.energy_producer_gradients.view();

    ComputeGradientHessianInfo this_info{
        info.dt,
        global_gradient_view.subview(m_impl.gradient_offset, m_impl.gradient_count),
        info.hessians.subview(m_impl.hessian_offset, m_impl.hessian_count)};

    do_compute_gradient_hessian(this_info);
}

void FiniteElementEnergyProducer::ReportExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}

void FiniteElementEnergyProducer::ReportExtentInfo::gradient_count(SizeT count) noexcept
{
    m_gradient_count = count;
}

void FiniteElementEnergyProducer::ReportExtentInfo::hessian_count(SizeT count) noexcept
{
    m_hessian_count = count;
}
}  // namespace uipc::backend::cuda
