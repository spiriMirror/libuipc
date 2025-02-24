#include <linear_system/diag_linear_subsystem.h>

namespace uipc::backend::cuda
{
void DiagLinearSubsystem::do_build(BuildInfo& info) {}
void DiagLinearSubsystem::do_build()
{
    auto& global_linear_system = require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    global_linear_system.add_subsystem(this);
}
void DiagLinearSubsystem::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    do_report_init_extent(info);
}
void DiagLinearSubsystem::receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    do_receive_init_dof_info(info);
}
void DiagLinearSubsystem::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    do_report_extent(info);
}
void DiagLinearSubsystem::assemble(GlobalLinearSystem::DiagInfo& info)
{
    do_assemble(info);
}
void DiagLinearSubsystem::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    do_accuracy_check(info);
}
void DiagLinearSubsystem::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    do_retrieve_solution(info);
}
}  // namespace uipc::backend::cuda
