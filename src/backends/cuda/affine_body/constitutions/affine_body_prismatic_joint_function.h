#pragma once
#include <utils/make_spd.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::affine_body_prismatic_joint
{
#include "sym/affine_body_prismatic_joint.inl"
}
namespace sym::affine_body_driving_prismatic_joint
{
#include "sym/affine_body_driving_prismatic_joint.inl"
}

MUDA_DEVICE MUDA_INLINE void compute_absolute_distance(Float& out_distance,
                                                       const Vector6&  C_bar,
                                                       const Vector6&  t_bar,
                                                       const Vector12& q_i,
                                                       const Vector12& q_j)
{
    Vector9 F01_q;
    namespace DPJ = uipc::backend::cuda::sym::affine_body_driving_prismatic_joint;

    DPJ::F01_q<Float>(F01_q,
                      C_bar.segment<3>(0),
                      t_bar.segment<3>(0),
                      q_i,
                      C_bar.segment<3>(3),
                      t_bar.segment<3>(3),
                      q_j);
    DPJ::Distance<Float>(out_distance, F01_q);
}

MUDA_DEVICE MUDA_INLINE void compute_absolute_distance_derivative(Eigen::Vector<Float, 24>& out_dx_dq,
                                                                  const Vector6& C_bar,
                                                                  const Vector6& t_bar,
                                                                  const Vector12& q_i,
                                                                  const Vector12& q_j)
{
    Vector9 F01_q;
    namespace DPJ = uipc::backend::cuda::sym::affine_body_driving_prismatic_joint;
    DPJ::F01_q<Float>(F01_q,
                      C_bar.segment<3>(0),
                      t_bar.segment<3>(0),
                      q_i,
                      C_bar.segment<3>(3),
                      t_bar.segment<3>(3),
                      q_j);

    Eigen::Vector<Float, 9> dxdF01;
    DPJ::dxdF01<Float>(dxdF01, F01_q);


    DPJ::J01T_G01<Float>(out_dx_dq,
                         dxdF01,
                         C_bar.segment<3>(0),
                         t_bar.segment<3>(0),
                         C_bar.segment<3>(3),
                         t_bar.segment<3>(3));
}

MUDA_DEVICE MUDA_INLINE void compute_absolute_distance_hessian(Eigen::Matrix<Float, 24, 24>& out_H,
                                                               const Float& dE_dx,
                                                               const Vector6& C_bar,
                                                               const Vector6& t_bar,
                                                               const Vector12& q_i,
                                                               const Vector12& q_j)
{
    Vector9 F01_q;
    namespace DPJ = uipc::backend::cuda::sym::affine_body_driving_prismatic_joint;
    DPJ::F01_q<Float>(F01_q,
                      C_bar.segment<3>(0),
                      t_bar.segment<3>(0),
                      q_i,
                      C_bar.segment<3>(3),
                      t_bar.segment<3>(3),
                      q_j);


    Matrix9x9 ddx_ddF;
    DPJ::ddxdF01(ddx_ddF, F01_q);
    Matrix9x9 H_D = dE_dx * ddx_ddF;
    make_spd(H_D);

    Eigen::Matrix<Float, 24, 24> JT_H_J;
    DPJ::J01T_H01_J01<Float>(JT_H_J,
                             H_D,
                             C_bar.segment<3>(0),
                             t_bar.segment<3>(0),
                             C_bar.segment<3>(3),
                             t_bar.segment<3>(3));

    out_H += JT_H_J;
}
}  // namespace uipc::backend::cuda