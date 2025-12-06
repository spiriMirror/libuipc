#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/affine_body_external_force_manager.h>

namespace uipc::backend::cuda
{
void AffineBodyExternalForceReporter::do_build()
{
    BuildInfo info;
    do_build(info);

    // Register with AffineBodyExternalForceManager
    auto& manager = require<AffineBodyExternalForceManager>();
    manager.register_subreporter(this);
}

U64 AffineBodyExternalForceReporter::uid() const noexcept
{
    return get_uid();
}

void AffineBodyExternalForceReporter::init()
{
    do_init();
}

void AffineBodyExternalForceReporter::step()
{
    do_step();
}
}  // namespace uipc::backend::cuda
