#include <finite_element/fem_line_search_reporter.h>
#include <finite_element/fem_line_search_subreporter.h>
#include <finite_element/finite_element_kinetic.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMLineSearchReporter);

void FEMLineSearchReporter::do_init(InitInfo& info)
{
    m_impl.init(info);
}

void FEMLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();
}

void FEMLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    m_impl.record_start_point(info);
}

void FEMLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    m_impl.step_forward(info);
}

void FEMLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
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

void FEMLineSearchReporter::Impl::compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace muda;

    // Compute Kinetic (special)
    {
        auto vertex_count = fem().xs.size();
        kinetic_energies.resize(vertex_count);
        auto kinetic_info = ComputeEnergyInfo{kinetic_energies.view(), info.dt()};
        finite_element_kinetic->compute_energy(kinetic_info);

        DeviceReduce().Sum(kinetic_energies.data(),
                           total_kinetic_energy.data(),
                           kinetic_energies.size());
    }

    // Collect the energy from all reporters
    {
        auto         reporter_view = reporters.view();
        span<IndexT> counts        = reporter_energy_offsets_counts.counts();
        for(auto&& [i, R] : enumerate(reporter_view))
        {
            ReportExtentInfo this_info;
            R->report_extent(this_info);
            counts[i] = this_info.m_energy_count;
        }

        reporter_energy_offsets_counts.scan();
        reporter_energies.resize(reporter_energy_offsets_counts.total_count());

        for(auto&& [i, R] : enumerate(reporter_view))
        {
            auto [offset, count] = reporter_energy_offsets_counts[i];
            auto this_info =
                ComputeEnergyInfo{reporter_energies.view(offset, count), info.dt()};
            R->compute_energy(this_info);
        }

        DeviceReduce().Sum(reporter_energies.data(),
                           total_reporter_energy.data(),
                           reporter_energies.size());
    }

    Float K       = total_kinetic_energy;
    Float other_E = total_reporter_energy;
    Float total_E = K + other_E;

    info.energy(total_E);
}

void FEMLineSearchReporter::Impl::init(LineSearchReporter::InitInfo& info)
{
    kinetic_energies.resize(fem().xs.size());

    auto reporter_view = reporters.view();
    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;
    for(auto&& [i, R] : enumerate(reporter_view))
        R->init();

    reporter_energy_offsets_counts.resize(reporter_view.size());
}

void FEMLineSearchReporter::add_reporter(FEMLineSearchSubreporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter is null");
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.reporters.register_sim_system(*reporter);
}

void FEMLineSearchReporter::add_kinetic(FiniteElementKinetic* kinetic)
{
    UIPC_ASSERT(kinetic, "kinetic is null");
    check_state(SimEngineState::BuildSystems, "add_kinetic()");
    m_impl.finite_element_kinetic.register_sim_system(*kinetic);
}
}  // namespace uipc::backend::cuda
