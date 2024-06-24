#include <linear_system/local_preconditioner.h>

namespace uipc::backend::cuda
{
void LocalPreconditioner::assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info)
{
    do_assemble(info);
}
void LocalPreconditioner::apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    do_apply(info);
}
}  // namespace uipc::backend::cuda
