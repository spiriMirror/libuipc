#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/log.h>
namespace uipc::backend::cuda
{
void OffDiagLinearSubsystem::depend_on(DiagLinearSubsystem* L, DiagLinearSubsystem* R)
{
    m_l = L;
    m_r = R;
}
void OffDiagLinearSubsystem::do_build()
{
    auto&     global_linear_system = require<GlobalLinearSystem>();
    BuildInfo info;
    do_build(info);

    UIPC_ASSERT(info.m_diag_l != nullptr && info.m_diag_r != nullptr,
                "Did you forget to call BuildInfo::connect() in {}'s do_build()?",
                this->name());

    global_linear_system.add_subsystem(this, info.m_diag_l, info.m_diag_r);
}
}  // namespace uipc::backend::cuda
