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

void ContactLineSearchReporter::Impl::init()
{
    auto reporters = global_contact_manager->m_impl.contact_reporters.view();
    contact_energies.resize(reporters.size(), 0);
    h_contact_energies.resize(reporters.size(), 0);
}

void ContactLineSearchReporter::Impl::do_compute_energy(LineSearcher::EnergyInfo& info)
{
    auto reporters = global_contact_manager->m_impl.contact_reporters.view();
    for(auto&& [i, reporter] : enumerate(reporters))
    {
        GlobalContactManager::EnergyInfo this_info;
        this_info.m_energy = muda::VarView<Float>{contact_energies.data() + i};
        this_info.m_is_initial = info.is_initial();
        reporter->compute_energy(this_info);
    }

    contact_energies.view().copy_to(h_contact_energies.data());

    Float total_contact_energy =
        std::accumulate(h_contact_energies.begin(), h_contact_energies.end(), 0.0f);

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
