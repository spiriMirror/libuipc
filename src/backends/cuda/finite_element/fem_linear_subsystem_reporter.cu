#include <finite_element/fem_linear_subsystem_reporter.h>

namespace uipc::backend::cuda
{
void FEMLinearSubsystemReporter::do_build()
{
    auto& fem_linear_subsystem = require<FEMLinearSubsystem>();

    // let subclass do the actual build
    BuildInfo info;
    do_build(info);

    fem_linear_subsystem.add_reporter(this);
}

void FEMLinearSubsystemReporter::init()
{
    InitInfo info;
    do_init(info);
}

void FEMLinearSubsystemReporter::report_extent(ReportExtentInfo& info)
{
    do_report_extent(info);
}

void FEMLinearSubsystemReporter::assemble(AssembleInfo& info)
{
    do_assemble(info);
}
}  // namespace uipc::backend::cuda
