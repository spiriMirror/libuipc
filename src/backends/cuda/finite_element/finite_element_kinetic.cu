#include <finite_element/finite_element_kinetic.h>
#include <finite_element/finite_element_diff_dof_reporter.h>
#include <finite_element/fem_linear_subsystem.h>
#include <finite_element/fem_line_search_reporter.h>

namespace uipc::backend::cuda
{
void FiniteElementKinetic::do_build()
{
    // provide fem data for solving
    m_impl.finite_element_method = require<FiniteElementMethod>();

    auto& linear_system = require<FEMLinearSubsystem>();
    auto& line_searcher = require<FEMLineSearchReporter>();

    BuildInfo this_info;
    do_build(this_info);

    line_searcher.add_kinetic(this);
    linear_system.add_kinetic(this);
}

void FiniteElementKinetic::compute_energy(FEMLineSearchReporter::ComputeEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl, &info};
    do_compute_energy(this_info);
}

void FiniteElementKinetic::compute_gradient_hessian(FEMLinearSubsystem::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{&m_impl, &info};
    do_compute_gradient_hessian(this_info);
}
}  // namespace uipc::backend::cuda
