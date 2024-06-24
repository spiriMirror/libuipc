#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/log.h>
namespace uipc::backend::cuda
{
void OffDiagLinearSubsystem::depend_on(DiagLinearSubsystem* L, DiagLinearSubsystem* R)
{
    m_l = L;
    m_r = R;
}
void OffDiagLinearSubsystem::check_dep_systems() const
{
    UIPC_ASSERT(m_l && m_r, "Dependent DiagLinearSubsystems are nullptr, call `depend_on()` in `do_build()` to setup them");
}
}  // namespace uipc::backend::cuda
