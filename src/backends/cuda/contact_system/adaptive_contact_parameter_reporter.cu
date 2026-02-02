#include <contact_system/adaptive_contact_parameter_reporter.h>
#include <contact_system/global_contact_manager.h>

namespace uipc::backend::cuda
{
void AdaptiveContactParameterReporter::do_build()
{
    auto& manager = require<GlobalContactManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}

void AdaptiveContactParameterReporter::init()
{
    InitInfo info;
    do_init(info);
}

void AdaptiveContactParameterReporter::compute_parameters(AdaptiveParameterInfo& info)
{
    do_compute_parameters(info);
}
}  // namespace uipc::backend::cuda
