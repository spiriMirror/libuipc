#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
MUDA_GENERIC Matrix3x3 q_to_A(const Vector12& q);
MUDA_GENERIC Vector9   A_to_q(const Matrix3x3& A);

MUDA_GENERIC Vector9   F_to_A(const Vector9& F);
MUDA_GENERIC Matrix9x9 HF_to_HA(const Matrix9x9& HF);
}
