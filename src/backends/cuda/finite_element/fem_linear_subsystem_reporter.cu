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
    info.check(name());

    UIPC_ASSERT(!(info.gradient_only() && info.m_hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name(),
                info.m_hessian_count);
}

void FEMLinearSubsystemReporter::assemble(AssembleInfo& info)
{
    do_assemble(info);
}
}  // namespace uipc::backend::cuda
