#include <finite_element/fem_line_search_subreporter.h>

namespace uipc::backend::cuda
{
void FEMLineSearchSubreporter::do_build()
{
    auto& fem_line_search = require<FEMLineSearchReporter>();

    // let subclass do the actual build
    BuildInfo info;
    do_build(info);

    fem_line_search.add_reporter(this);
}

void FEMLineSearchSubreporter::init()
{
    InitInfo info;
    do_init(info);
}

void FEMLineSearchSubreporter::report_extent(ReportExtentInfo& info)
{
    do_report_extent(info);
}

void FEMLineSearchSubreporter::compute_energy(ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}
}  // namespace uipc::backend::cuda
