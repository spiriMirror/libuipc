#include <contact_system/contact_line_search_reporter.h>
#include <contact_system/global_contact_manager.h>
namespace uipc::backend::cuda
{
void ContactLineSearchReporter::do_build() 
{
    m_impl.global_contact_manager = find<GlobalContactManager>();
}

void ContactLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    // Do nothing
}
void ContactLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    // Do nothing
}
void ContactLineSearchReporter::do_compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
}
}  // namespace uipc::backend::cuda
