#include <finite_element/fem_line_search_reporter.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMLineSearchReporter);

void FEMLineSearchReporter::do_init(InitInfo& info) {}

void FEMLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();

    auto fea = find<FiniteElementAnimator>();
    if(fea)
        m_impl.finite_element_animator = *fea;
}

void FEMLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    m_impl.record_start_point(info);
}

void FEMLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    m_impl.step_forward(info);
}

void FEMLineSearchReporter::do_compute_energy(LineSearcher::EnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void FEMLineSearchReporter::Impl::record_start_point(LineSearcher::RecordInfo& info)
{
    using namespace muda;

    fem().x_temps = fem().xs;
}

void FEMLineSearchReporter::Impl::step_forward(LineSearcher::StepInfo& info)
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(fem().xs.size(),
               [is_fixed = fem().is_fixed.cviewer().name("is_fixed"),
                x_temps  = fem().x_temps.cviewer().name("x_temps"),
                xs       = fem().xs.viewer().name("xs"),
                dxs      = fem().dxs.cviewer().name("dxs"),
                alpha    = info.alpha] __device__(int i) mutable
               { xs(i) = x_temps(i) + alpha * dxs(i); });
}

void FEMLineSearchReporter::Impl::compute_energy(LineSearcher::EnergyInfo& info)
{
    using namespace muda;

    // Kinetic/Elastic/Contact ...
    for(auto* producer : fem().energy_producers)
        producer->compute_energy(info);

    DeviceReduce().Sum(fem().energy_producer_energies.data(),
                       fem().energy_producer_energy.data(),
                       fem().energy_producer_energies.size());

    // copy back to host
    Float E = fem().energy_producer_energy;

    // Animation
    Float anim_E = 0.0;
    if(finite_element_animator)
        anim_E = finite_element_animator->compute_energy(info);

    Float total_E = E + anim_E;

    info.energy(total_E);
}
}  // namespace uipc::backend::cuda
