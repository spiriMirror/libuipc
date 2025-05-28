#include <linear_system/local_preconditioner.h>

namespace uipc::backend::cuda
{
void LocalPreconditioner::do_build()
{
    auto& global_linear_system = require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_subsystem = info.m_subsystem;
    UIPC_ASSERT(m_subsystem != nullptr, "The depend_subsystem should not be nullptr.");

    global_linear_system.add_preconditioner(this);
}

void LocalPreconditioner::init()
{
    InitInfo info;
    do_init(info);
}

void LocalPreconditioner::assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info)
{
    do_assemble(info);
}

void LocalPreconditioner::apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    do_apply(info);
}

void LocalPreconditioner::BuildInfo::connect(DiagLinearSubsystem* system)
{
    m_subsystem = system;
}
}  // namespace uipc::backend::cuda
