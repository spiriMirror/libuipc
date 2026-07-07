#pragma once
#include <type_define.h>
#include <numbers>

namespace uipc::backend::cuda
{
// Unwrap `wrapped_angle` to the 2*pi representative nearest `ref` (energy and gradient must pass identical args).
UIPC_GENERIC inline Float unwrap_angle(Float wrapped_angle, Float ref)
{
    return ref + ::remainder(wrapped_angle - ref, 2.0 * std::numbers::pi);
}

namespace sym::affine_body_revolute_joint
{
#include "sym/affine_body_revolute_joint.inl"
}
namespace sym::affine_body_driving_revolute_joint
{
#include "sym/affine_body_driving_revolute_joint.inl"
}

namespace sym::affine_body_revolute_joint_limit
{
#include <affine_body/constitutions/sym/affine_body_revolute_joint_limit.inl>
}

MUDA_DEVICE MUDA_INLINE void compute_relative_angle(Float&          out_theta,
                                                    const Vector6&  l_basis,
                                                    const Vector12& q_i,
                                                    const Vector6&  r_basis,
                                                    const Vector12& q_j)
{
    Vector12 F01_q;
    Vector3  ni_bar = l_basis.segment<3>(0);
    Vector3  bi_bar = l_basis.segment<3>(3);
    Vector3  nj_bar = r_basis.segment<3>(0);
    Vector3  bj_bar = r_basis.segment<3>(3);
    sym::affine_body_driving_revolute_joint::F01_q<Float>(
        F01_q, ni_bar, bi_bar, q_i, nj_bar, bj_bar, q_j);
    sym::affine_body_driving_revolute_joint::theta<Float>(out_theta, F01_q);
}

// Fold `aim_angle` into a theta_tilde matching the wrapped measurement (energy and gradient must pass identical args).
MUDA_DEVICE MUDA_INLINE Float driving_theta_tilde(const Vector12& F01_q,
                                                  Float           aim_angle,
                                                  Float           init_angle,
                                                  Float           ref_angle)
{
    Float theta_a;
    sym::affine_body_driving_revolute_joint::theta<Float>(theta_a, F01_q);
    Float theta_eff = unwrap_angle(theta_a, ref_angle - init_angle);
    return aim_angle - init_angle - (theta_eff - theta_a);
}
}  // namespace uipc::backend::cuda