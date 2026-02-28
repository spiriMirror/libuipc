#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
UIPC_GENERIC Matrix3x3 q_to_A(const Vector12& q);
UIPC_GENERIC Vector9   A_to_q(const Matrix3x3& A);

UIPC_GENERIC Vector9   F_to_A(const Vector9& F);
UIPC_GENERIC Matrix9x9 HF_to_HA(const Matrix9x9& HF);

UIPC_GENERIC Matrix4x4 q_to_transform(const Vector12& q);
UIPC_GENERIC Vector12  transform_to_q(const Matrix4x4& transform);

UIPC_GENERIC Matrix4x4 q_v_to_transform_v(const Vector12& q);
UIPC_GENERIC Vector12  transform_v_to_q_v(const Matrix4x4& transform_v);
}  // namespace uipc::backend::cuda
