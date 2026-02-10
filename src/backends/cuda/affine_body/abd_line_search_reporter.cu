#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/affine_body_constitution.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <affine_body/abd_line_search_subreporter.h>
#include <affine_body/affine_body_kinetic.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDLineSearchReporter);

void ABDLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
}

void ABDLineSearchReporter::Impl::init(LineSearchReporter::InitInfo& info)
{
    auto reporter_view = reporters.view();
    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;  // Assign index for each reporter
    for(auto&& [i, R] : enumerate(reporter_view))
        R->init();

    reporter_energy_offsets_counts.resize(reporter_view.size());
}

void ABDLineSearchReporter::Impl::record_start_point(LineSearcher::RecordInfo& info)
{
    using namespace muda;

    BufferLaunch().template copy<Vector12>(abd().body_id_to_q_temp.view(),
                                           abd().body_id_to_q.view());
}

void ABDLineSearchReporter::Impl::step_forward(LineSearcher::StepInfo& info)
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(abd().abd_body_count,
               [is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                q_temps  = abd().body_id_to_q_temp.cviewer().name("q_temps"),
                qs       = abd().body_id_to_q.viewer().name("qs"),
                dqs      = abd().body_id_to_dq.cviewer().name("dqs"),
                alpha    = info.alpha] __device__(int i) mutable
               {
                   if(is_fixed(i))
                       return;
                   qs(i) = q_temps(i) + alpha * dqs(i);
               });
}

void ABDLineSearchReporter::Impl::compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace muda;

    auto body_count = abd().body_count();

    // Compute kinetic energy
    {
        body_id_to_kinetic_energy.resize(body_count);

        ABDLineSearchReporter::ComputeEnergyInfo this_info;
        this_info.m_energies = body_id_to_kinetic_energy.view();
        this_info.m_dt       = info.dt();

        abd().kinetic->compute_energy(this_info);

        using namespace muda;

        // Zero out the kinetic energy of fixed bodies and bodies with external kinetic
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(abd().abd_body_count,
                   [is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                    external_kinetic =
                        abd().body_id_to_external_kinetic.cviewer().name("external_kinetic"),
                    kinetic_energy = body_id_to_kinetic_energy.viewer().name(
                        "kinetic_energy")] __device__(int i) mutable
                   {
                       if(is_fixed(i) || external_kinetic(i))
                       {
                           kinetic_energy(i) = 0.0;
                       }
                   });

        // Sum up the kinetic energy
        DeviceReduce().Sum(body_id_to_kinetic_energy.data(),
                           abd_kinetic_energy.data(),
                           body_id_to_kinetic_energy.size());
    }

    // Compute shape energy
    {
        body_id_to_shape_energy.resize(body_count);

        // Distribute the computation of shape energy to each constitution
        for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
        {
            auto shape_energy = abd().subview(body_id_to_shape_energy, cst->m_index);

            ABDLineSearchReporter::ComputeEnergyInfo this_info;
            this_info.m_energies = shape_energy;
            this_info.m_dt       = info.dt();
            cst->compute_energy(this_info);
        }

        // Sum up the shape energy
        DeviceReduce().Sum(body_id_to_shape_energy.data(),
                           abd_shape_energy.data(),
                           body_id_to_shape_energy.size());
    }

    // Collect the energy from other reporters
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
            ComputeEnergyInfo this_info;
            auto [offset, count] = reporter_energy_offsets_counts[i];
            this_info.m_energies = reporter_energies.view(offset, count);
            this_info.m_dt       = info.dt();
            R->compute_energy(this_info);
        }

        // Compute the total energy from all reporters
        DeviceReduce().Sum(reporter_energies.data(),
                           total_reporter_energy.data(),
                           reporter_energies.size());
    }

    // Copy from device to host
    Float K       = abd_kinetic_energy;
    Float shape_E = abd_shape_energy;
    Float other_E = total_reporter_energy;

    Float E = K + shape_E + other_E;

    info.energy(E);
}

void ABDLineSearchReporter::do_init(LineSearchReporter::InitInfo& info)
{
    m_impl.init(info);
}

void ABDLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    m_impl.record_start_point(info);
}

void ABDLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    m_impl.step_forward(info);
}

void ABDLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void ABDLineSearchReporter::add_reporter(ABDLineSearchSubreporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter is null");
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.reporters.register_sim_system(*reporter);
}
}  // namespace uipc::backend::cuda
