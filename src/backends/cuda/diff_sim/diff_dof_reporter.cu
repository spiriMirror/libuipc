#include <diff_sim/diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void DiffDofReporter::do_build()
{
    auto& manager = require<GlobalDiffSimManager>();

    BuildInfo info;
    do_build(info);

    manager.add_subsystem(this);
}
}  // namespace uipc::backend::cuda