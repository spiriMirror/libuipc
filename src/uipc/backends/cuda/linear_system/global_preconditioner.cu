#include <linear_system/global_preconditioner.h>

namespace uipc::backend::cuda
{
void GlobalPreconditioner::assemble(GlobalLinearSystem::GlobalPreconditionerAssemblyInfo& info)
{
    do_assemble(info);
}
void GlobalPreconditioner::apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    do_apply(info);
}
}  // namespace uipc::backend::cuda
