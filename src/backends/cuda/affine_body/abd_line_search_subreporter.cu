#include <affine_body/abd_line_search_subreporter.h>

namespace uipc::backend::cuda
{
void ABDLineSearchSubreporter::do_build()
{
    auto& abd_line_search = require<ABDLineSearchReporter>();

    // let subclass do the actual build
    BuildInfo info;
    do_build(info);

    abd_line_search.add_reporter(this);
}

void ABDLineSearchSubreporter::init()
{
    InitInfo info;
    do_init(info);
}
void ABDLineSearchSubreporter::report_extent(ReportExtentInfo& info)
{
    do_report_extent(info);
}

void ABDLineSearchSubreporter::compute_energy(ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}
}  // namespace uipc::backend::cuda