#include <line_search/line_search_reporter.h>
#include <typeinfo>

namespace uipc::backend::cuda
{
void LineSearchReporter::do_build(BuildInfo& info) {}
void LineSearchReporter::do_build()
{
    auto& line_searcher = require<LineSearcher>();
    line_searcher.add_reporter(this);
    BuildInfo info;
    do_build(info);
}
void LineSearchReporter::record_start_point(LineSearcher::RecordInfo& info)
{
    do_record_start_point(info);
}
void LineSearchReporter::step_forward(LineSearcher::StepInfo& info)
{
    do_step_forward(info);
}
void LineSearchReporter::compute_energy(LineSearcher::EnergyInfo& info)
{
    do_compute_energy(info);
}
}  // namespace uipc::backend::cuda
