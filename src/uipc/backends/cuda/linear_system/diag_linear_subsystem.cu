#include <linear_system/diag_linear_subsystem.h>

namespace uipc::backend::cuda
{
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
