#include <linear_system/global_preconditioner.h>

namespace uipc::backend::cuda
{
void GlobalPreconditioner::do_build() 
{
    auto& global_linear_system = require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    global_linear_system.add_preconditioner(this);
}
void GlobalPreconditioner::assemble(GlobalLinearSystem::GlobalPreconditionerAssemblyInfo& info)
{
    do_assemble(info);
}
void GlobalPreconditioner::apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    do_apply(info);
}
}  // namespace uipc::backend::cuda
