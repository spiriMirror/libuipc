#include <diff_sim/diff_sim_subsystem.h>

namespace uipc::backend::cuda
{
void DiffSimSubsystem::do_build()
{
    auto& manager = require<GlobalDiffSimManager>();

    BuildInfo info;
    do_build(info);

    manager.add_subsystem(this);
}
}  // namespace uipc::backend::cuda