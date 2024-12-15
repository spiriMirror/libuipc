#include <diff_sim/diff_parm_reporter.h>

namespace uipc::backend::cuda
{
void DiffParmReporter::do_build()
{
    auto& manager = require<GlobalDiffSimManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}
void DiffParmReporter::report_extent(GlobalDiffSimManager::DiffParmExtentInfo& info)
{
    do_report_extent(info);
}

void DiffParmReporter::assemble_diff_parm(GlobalDiffSimManager::DiffParmInfo& info)
{
    do_assemble(info);
}
void DiffParmReporter::update_diff_parm(GlobalDiffSimManager::DiffParmUpdateInfo& info)
{
    do_update(info);
}
}  // namespace uipc::backend::cuda