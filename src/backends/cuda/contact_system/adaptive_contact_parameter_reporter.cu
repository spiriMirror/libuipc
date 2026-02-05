#include <contact_system/adaptive_contact_parameter_reporter.h>
#include <contact_system/global_contact_manager.h>

namespace uipc::backend::cuda
{
void AdaptiveContactParameterReporter::do_build()
{
    auto resistance =
        world().scene().contact_tabular().contact_models().find<Float>("resistance");
    auto resistance_view = resistance->view();
    auto min_kappa       = std::ranges::min(resistance_view);
    if(min_kappa > 0.0)
    {
        throw SimSystemException("No Adaptive Contact Parameters Found");
    }

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
