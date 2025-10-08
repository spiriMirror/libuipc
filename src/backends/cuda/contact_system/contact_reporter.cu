#include <contact_system/contact_reporter.h>

namespace uipc::backend::cuda
{
void ContactReporter::do_init(InitInfo&) {}

void ContactReporter::init()
{
    InitInfo info;
    do_init(info);
}

void ContactReporter::do_build(DyTopoEffectReporter::BuildInfo& info)
{
    auto& manager = require<GlobalContactManager>();

    BuildInfo this_info;
    do_build(this_info);

    manager.add_reporter(this);
}
}  // namespace uipc::backend::cuda
