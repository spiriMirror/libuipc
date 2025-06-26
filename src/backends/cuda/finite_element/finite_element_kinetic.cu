#include <finite_element/finite_element_kinetic.h>
#include <finite_element/finite_element_diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void FiniteElementKinetic::do_build(FiniteElementEnergyProducer::BuildInfo& info)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();

    BuildInfo this_info;
    do_build(this_info);

    m_impl.finite_element_method->add_kinetic(this);
}

void FiniteElementKinetic::do_report_extent(ReportExtentInfo& info)
{
    auto vert_count = m_impl.finite_element_method->xs().size();
    info.energy_count(vert_count);
    info.stencil_dim(1);
}

void FiniteElementKinetic::do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl, &info};
    do_compute_energy(this_info);
}

void FiniteElementKinetic::do_compute_gradient_hessian(FiniteElementEnergyProducer::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{&m_impl, &info};
    do_compute_gradient_hessian(this_info);
}
}  // namespace uipc::backend::cuda
