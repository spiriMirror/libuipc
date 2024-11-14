#include <diff_sim/diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void DiffDofReporter::do_build()
{
    auto& manager = require<GlobalDiffSimManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}

void DiffDofReporter::report_extent(GlobalDiffSimManager::DiffDofExtentInfo& info)
{
    do_report_extent(info);
}

void DiffDofReporter::assemble_diff_dof(GlobalDiffSimManager::DiffDofInfo& info)
{
    do_assemble(info);
}
}  // namespace uipc::backend::cuda