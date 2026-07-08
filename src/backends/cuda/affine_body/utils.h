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

UIPC_GENERIC void orthonormal_basis(Vector3& t, Vector3& n, Vector3& b);

// Skew-symmetric matrix [v]_x such that [v]_x * w = v.cross(w).
UIPC_GENERIC Matrix3x3 skew(const Vector3& v);

// Convert a scalar torque `tau` about the world-space unit axis `e` into an
// ABD generalized force on the body with DOF `q` (virtual-work form):
//   F^A = (tau/2) * [e]_x * A^{-T}   (3x3), packed into the rotational
// segment of the returned Vector12; the translational segment is zero.
UIPC_GENERIC Vector12 torque_to_F(Float tau, const Vector3& e, const Vector12& q);
}  // namespace uipc::backend::cuda
