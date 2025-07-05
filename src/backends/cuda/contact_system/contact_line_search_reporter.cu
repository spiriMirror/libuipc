#include <contact_system/contact_line_search_reporter.h>
#include <contact_system/global_contact_manager.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ContactLineSearchReporter);

void ContactLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.global_contact_manager = require<GlobalContactManager>();
}

void ContactLineSearchReporter::do_init(LineSearchReporter::InitInfo& info)
{
    m_impl.init();
}

void ContactLineSearchReporter::Impl::init() {}

void ContactLineSearchReporter::Impl::do_compute_energy(LineSearcher::EnergyInfo& info)
{
    auto& gcm       = global_contact_manager->m_impl;
    auto  reporters = gcm.contact_reporters.view();

    auto energy_counts = gcm.reporter_energy_offsets_counts.counts();
    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalContactManager::EnergyExtentInfo extent_info;
        reporter->report_energy_extent(extent_info);
        energy_counts[i] = extent_info.m_energy_count;
    }

    gcm.reporter_energy_offsets_counts.scan();
    energies.resize(gcm.reporter_energy_offsets_counts.total_count());

    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalContactManager::EnergyInfo this_info;
        auto [offset, count]   = gcm.reporter_energy_offsets_counts[i];
        this_info.m_energies   = energies.view(offset, count);
        this_info.m_is_initial = info.is_initial();
        reporter->compute_energy(this_info);
    }

    using namespace muda;

    DeviceReduce().Sum(energies.data(), energy.data(), energies.size());

    // Copy from device to host
    Float total_contact_energy = energy;

    info.energy(total_contact_energy);
}

void ContactLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    // Do nothing, because GlobalVertexManager will do the record start point for all the vertices we need
}

void ContactLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    // Do nothing, because GlobalVertexManager will do the step forward for all the vertices we need
}

void ContactLineSearchReporter::do_compute_energy(LineSearcher::EnergyInfo& info)
{
    m_impl.do_compute_energy(info);
}
}  // namespace uipc::backend::cuda
