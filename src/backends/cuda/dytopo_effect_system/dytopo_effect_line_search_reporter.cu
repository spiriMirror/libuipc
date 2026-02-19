#include <dytopo_effect_system/dytopo_effect_line_search_reporter.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DyTopoEffectLineSearchReporter);

void DyTopoEffectLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{
    m_impl.global_dytopo_effect_manager = require<GlobalDyTopoEffectManager>();
}

void DyTopoEffectLineSearchReporter::do_init(LineSearchReporter::InitInfo& info)
{
    m_impl.init();
}

void DyTopoEffectLineSearchReporter::Impl::init() {}

void DyTopoEffectLineSearchReporter::Impl::compute_energy(bool is_init)
{
    auto& manager   = global_dytopo_effect_manager->m_impl;
    auto  reporters = manager.dytopo_effect_reporters.view();

    auto energy_counts = manager.reporter_energy_offsets_counts.counts();
    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalDyTopoEffectManager::EnergyExtentInfo extent_info;
        reporter->report_energy_extent(extent_info);
        energy_counts[i] = extent_info.m_energy_count;
    }

    manager.reporter_energy_offsets_counts.scan();
    energies.resize(manager.reporter_energy_offsets_counts.total_count());

    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalDyTopoEffectManager::EnergyInfo this_info;
        auto [offset, count]   = manager.reporter_energy_offsets_counts[i];
        this_info.m_energies   = energies.view(offset, count);
        this_info.m_is_initial = is_init;
        reporter->compute_energy(this_info);
    }
}

void DyTopoEffectLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    // Do nothing, because GlobalVertexManager will do the record start point for all the vertices we need
}

void DyTopoEffectLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    // Do nothing, because GlobalVertexManager will do the step forward for all the vertices we need
}

void DyTopoEffectLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    using namespace muda;

    m_impl.compute_energy(info.is_initial());

    DeviceReduce().Sum(
        m_impl.energies.data(), m_impl.energy.data(), m_impl.energies.size());

    info.energy(m_impl.energy);
}
}  // namespace uipc::backend::cuda
