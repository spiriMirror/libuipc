#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/log.h>
namespace uipc::backend::cuda
{
void OffDiagLinearSubsystem::depend_on(DiagLinearSubsystem* L, DiagLinearSubsystem* R)
{
    m_l = L;
    m_r = R;
}
}  // namespace uipc::backend::cuda
