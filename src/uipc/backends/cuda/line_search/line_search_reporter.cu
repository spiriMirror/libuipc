#include <line_search/line_search_reporter.h>
#include <typeinfo>

namespace uipc::backend::cuda
{
std::string_view LineSearchReporter::get_name() const noexcept
{
    return typeid(*this).name();
}
void LineSearchReporter::record_start_point(LineSearcher::RecordInfo& info)
{
    do_record_start_point(info);
}
void LineSearchReporter::step_forward(LineSearcher::StepInfo& info)
{
    do_step_forward(info);
}
void LineSearchReporter::compute_energy(LineSearcher::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}
std::string_view LineSearchReporter::name() const noexcept
{
    return get_name();
}
}  // namespace uipc::backend::cuda
