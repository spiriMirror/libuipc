#include <affine_body/abd_linear_subsystem_reporter.h>

namespace uipc::backend::cuda
{
void ABDLinearSubsystemReporter::do_build()
{
    auto& abd_linear_subsystem = require<ABDLinearSubsystem>();

    // let subclass do the actual build
    BuildInfo info;
    do_build(info);

    abd_linear_subsystem.add_reporter(this);
}

void ABDLinearSubsystemReporter::init()
{
    InitInfo info;
    do_init(info);
}

void ABDLinearSubsystemReporter::report_extent(ReportExtentInfo& info)
{
    do_report_extent(info);
    info.check(name());
}

void ABDLinearSubsystemReporter::assemble(AssembleInfo& info)
{
    do_assemble(info);
}
}  // namespace uipc::backend::cuda