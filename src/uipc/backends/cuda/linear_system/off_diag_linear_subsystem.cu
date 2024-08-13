#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/log.h>
namespace uipc::backend::cuda
{
void OffDiagLinearSubsystem::do_build()
{
    auto&     global_linear_system = require<GlobalLinearSystem>();
    BuildInfo info;
    do_build(info);

    UIPC_ASSERT(info.m_diag_l != nullptr && info.m_diag_r != nullptr,
                "Did you forget to call BuildInfo::connect() in {}'s do_build()?",
                this->name());

    m_l = info.m_diag_l;
    m_r = info.m_diag_r;

    global_linear_system.add_subsystem(this);
}
}  // namespace uipc::backend::cuda
