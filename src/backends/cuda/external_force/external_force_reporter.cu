#include <external_force/external_force_reporter.h>
#include <external_force/global_external_force_manager.h>

namespace uipc::backend::cuda
{
U64 ExternalForceReporter::uid() const noexcept
{
    return get_uid();
}

void ExternalForceReporter::do_build()
{
    BuildInfo info;
    do_build(info);

    // Register with GlobalExternalForceManager
    auto& manager = require<GlobalExternalForceManager>();
    manager.register_reporter(this);
}

void ExternalForceReporter::init()
{
    do_init();
}

void ExternalForceReporter::clear()
{
    do_clear();
}

void ExternalForceReporter::step()
{
    do_step();
}
}  // namespace uipc::backend::cuda
