#include <finite_element/finite_element_external_force_reporter.h>
#include <finite_element/finite_element_external_force_manager.h>

namespace uipc::backend::cuda
{
void FiniteElementExternalForceReporter::do_build()
{
    BuildInfo info;
    do_build(info);

    auto& manager = require<FEMExternalForceManager>();
    manager.register_reporter(this);
}

U64 FiniteElementExternalForceReporter::uid() const noexcept
{
    return get_uid();
}

void FiniteElementExternalForceReporter::init()
{
    do_init();
}

void FiniteElementExternalForceReporter::step(ExternalForceInfo& info)
{
    do_step(info);
}
}  // namespace uipc::backend::cuda
