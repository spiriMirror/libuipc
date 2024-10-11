#pragma once
#include <type_define.h>
#include <Eigen/Geometry>

// ref: https://github.com/ipc-sim/Codim-IPC/blob/main/Library/Math/DIHEDRAL_ANGLE.h
namespace uipc::backend::cuda
{
/**
 * @brief Compute the dihedral angle between two planes defined by four points.
 * 
 * Mid-Edge: V1-V2
 * Opposite Points: V0, V3
 */
template <typename T>
UIPC_GENERIC void dihedral_angle(const Eigen::Matrix<T, 3, 1>& v0,
                                 const Eigen::Matrix<T, 3, 1>& v1,
                                 const Eigen::Matrix<T, 3, 1>& v2,
                                 const Eigen::Matrix<T, 3, 1>& v3,
                                 T&                            DA)
{
    const Eigen::Matrix<T, 3, 1> n1 = (v1 - v0).cross(v2 - v0);
    const Eigen::Matrix<T, 3, 1> n2 = (v2 - v3).cross(v1 - v3);
    DA                              = std::acos(std::max(
        T(-1), std::min(T(1), n1.dot(n2) / std::sqrt(n1.squaredNorm() * n2.squaredNorm()))));
    if(n2.cross(n1).dot(v1 - v2) < 0)
    {
        DA = -DA;
    }
}

namespace detail
{
    // here we map our v order to rusmas' in this function for implementation convenience
    template <typename T>
    UIPC_GENERIC void dihedral_angle_gradient(const Eigen::Matrix<T, 3, 1>& v2,  // input: V0
                                              const Eigen::Matrix<T, 3, 1>& v0,  // input: V1
                                              const Eigen::Matrix<T, 3, 1>& v1,  // input: V2
                                              const Eigen::Matrix<T, 3, 1>& v3,  // input: V3
                                              Eigen::Matrix<T, 12, 1>& grad)
    {
        Eigen::Matrix<T, 3, 1> e0 = v1 - v0;
        Eigen::Matrix<T, 3, 1> e1 = v2 - v0;
        Eigen::Matrix<T, 3, 1> e2 = v3 - v0;
        Eigen::Matrix<T, 3, 1> e3 = v2 - v1;
        Eigen::Matrix<T, 3, 1> e4 = v3 - v1;
        Eigen::Matrix<T, 3, 1> n1 = e0.cross(e1);
        Eigen::Matrix<T, 3, 1> n2 = e2.cross(e0);

        T n1SqNorm = n1.squaredNorm();
        T n2SqNorm = n2.squaredNorm();
        T e0norm   = e0.norm();

        // fill in gradient in order with g2, g0, g1, g3 in rusmas' doc
        grad.template segment<3>(0) = -e0norm / n1SqNorm * n1;
        grad.template segment<3>(3) = -e0.dot(e3) / (e0norm * n1SqNorm) * n1
                                      - e0.dot(e4) / (e0norm * n2SqNorm) * n2;
        grad.template segment<3>(6) = e0.dot(e1) / (e0norm * n1SqNorm) * n1
                                      + e0.dot(e2) / (e0norm * n2SqNorm) * n2;
        grad.template segment<3>(9) = -e0norm / n2SqNorm * n2;
    }


    template <typename T>
    UIPC_GENERIC void compute_m_hat(const Eigen::Matrix<T, 3, 1>& xp,
                                    const Eigen::Matrix<T, 3, 1>& xe0,
                                    const Eigen::Matrix<T, 3, 1>& xe1,
                                    Eigen::Matrix<T, 3, 1>&       mHat)
    {
        Eigen::Matrix<T, 3, 1> e = xe1 - xe0;
        mHat = (xe0 + (xp - xe0).dot(e) / e.squaredNorm() * e - xp).normalized();
    }

    // here we map our v order to rusmas' in this function for implementation convenience
    template <typename T>
    UIPC_GENERIC void dihedral_angle_hessian(const Eigen::Matrix<T, 3, 1>& v2,  // input: V0
                                             const Eigen::Matrix<T, 3, 1>& v0,  // input: V1
                                             const Eigen::Matrix<T, 3, 1>& v1,  // input: V2
                                             const Eigen::Matrix<T, 3, 1>& v3,  // input: V3
                                             Eigen::Matrix<T, 12, 12>& Hess)
    {
        Eigen::Matrix<T, 3, 1> e[5] = {v1 - v0, v2 - v0, v3 - v0, v2 - v1, v3 - v1};
        T norm_e[5] = {
            e[0].norm(),
            e[1].norm(),
            e[2].norm(),
            e[3].norm(),
            e[4].norm(),
        };

        Eigen::Matrix<T, 3, 1> n1     = e[0].cross(e[1]);
        Eigen::Matrix<T, 3, 1> n2     = e[2].cross(e[0]);
        T                      n1norm = n1.norm();
        T                      n2norm = n2.norm();

        Eigen::Matrix<T, 3, 1> mHat1, mHat2, mHat3, mHat4, mHat01, mHat02;
        compute_m_hat(v1, v0, v2, mHat1);
        compute_m_hat(v1, v0, v3, mHat2);
        compute_m_hat(v0, v1, v2, mHat3);
        compute_m_hat(v0, v1, v3, mHat4);
        compute_m_hat(v2, v0, v1, mHat01);
        compute_m_hat(v3, v0, v1, mHat02);

        T cosalpha1, cosalpha2, cosalpha3, cosalpha4;
        cosalpha1 = e[0].dot(e[1]) / (norm_e[0] * norm_e[1]);
        cosalpha2 = e[0].dot(e[2]) / (norm_e[0] * norm_e[2]);
        cosalpha3 = -e[0].dot(e[3]) / (norm_e[0] * norm_e[3]);
        cosalpha4 = -e[0].dot(e[4]) / (norm_e[0] * norm_e[4]);

        T h1, h2, h3, h4, h01, h02;
        h1  = n1norm / norm_e[1];
        h2  = n2norm / norm_e[2];
        h3  = n1norm / norm_e[3];
        h4  = n2norm / norm_e[4];
        h01 = n1norm / norm_e[0];
        h02 = n2norm / norm_e[0];

        //TODO: can extract to functions
        Eigen::Matrix<T, 3, 3> N1_01 = n1 * (mHat01.transpose() / (h01 * h01 * n1norm));
        Eigen::Matrix<T, 3, 3> N1_3 = n1 * (mHat3.transpose() / (h01 * h3 * n1norm));
        Eigen::Matrix<T, 3, 3> N1_1 = n1 * (mHat1.transpose() / (h01 * h1 * n1norm));
        Eigen::Matrix<T, 3, 3> N2_4 = n2 * (mHat4.transpose() / (h02 * h4 * n2norm));
        Eigen::Matrix<T, 3, 3> N2_2 = n2 * (mHat2.transpose() / (h02 * h2 * n2norm));
        Eigen::Matrix<T, 3, 3> N2_02 = n2 * (mHat02.transpose() / (h02 * h02 * n2norm));
        Eigen::Matrix<T, 3, 3> M3_01_1 =
            (cosalpha3 / (h3 * h01 * n1norm) * mHat01) * n1.transpose();
        Eigen::Matrix<T, 3, 3> M1_01_1 =
            (cosalpha1 / (h1 * h01 * n1norm) * mHat01) * n1.transpose();
        Eigen::Matrix<T, 3, 3> M1_1_1 =
            (cosalpha1 / (h1 * h1 * n1norm) * mHat1) * n1.transpose();
        Eigen::Matrix<T, 3, 3> M3_3_1 =
            (cosalpha3 / (h3 * h3 * n1norm) * mHat3) * n1.transpose();
        Eigen::Matrix<T, 3, 3> M3_1_1 =
            (cosalpha3 / (h3 * h1 * n1norm) * mHat1) * n1.transpose();
        Eigen::Matrix<T, 3, 3> M1_3_1 =
            (cosalpha1 / (h1 * h3 * n1norm) * mHat3) * n1.transpose();
        Eigen::Matrix<T, 3, 3> M4_02_2 =
            (cosalpha4 / (h4 * h02 * n2norm) * mHat02) * n2.transpose();
        Eigen::Matrix<T, 3, 3> M2_02_2 =
            (cosalpha2 / (h2 * h02 * n2norm) * mHat02) * n2.transpose();
        Eigen::Matrix<T, 3, 3> M4_4_2 =
            (cosalpha4 / (h4 * h4 * n2norm) * mHat4) * n2.transpose();
        Eigen::Matrix<T, 3, 3> M2_4_2 =
            (cosalpha2 / (h2 * h4 * n2norm) * mHat4) * n2.transpose();
        Eigen::Matrix<T, 3, 3> M4_2_2 =
            (cosalpha4 / (h4 * h2 * n2norm) * mHat2) * n2.transpose();
        Eigen::Matrix<T, 3, 3> M2_2_2 =
            (cosalpha2 / (h2 * h2 * n2norm) * mHat2) * n2.transpose();
        Eigen::Matrix<T, 3, 3> B1 =
            n1 * (mHat01.transpose() / (norm_e[0] * norm_e[0] * n1norm));
        Eigen::Matrix<T, 3, 3> B2 =
            n2 * (mHat02.transpose() / (norm_e[0] * norm_e[0] * n2norm));

        // fill in Hessiann in order with g2, g0, g1, g3 in rusmus' doc
        Hess.template block<3, 3>(0, 0) = -(N1_01 + N1_01.transpose());
        Hess.template block<3, 3>(3, 0) = M3_01_1 - N1_3;
        Hess.template block<3, 3>(0, 3) = Hess.template block<3, 3>(3, 0).transpose();
        Hess.template block<3, 3>(6, 0) = M1_01_1 - N1_1;
        Hess.template block<3, 3>(0, 6) = Hess.template block<3, 3>(6, 0).transpose();
        Hess.template block<3, 3>(0, 9).setZero();
        Hess.template block<3, 3>(9, 0).setZero();

        Hess.template block<3, 3>(3, 3) =
            M3_3_1 + M3_3_1.transpose() - B1 + M4_4_2 + M4_4_2.transpose() - B2;
        Hess.template block<3, 3>(3, 6) =
            M3_1_1 + M1_3_1.transpose() + B1 + M4_2_2 + M2_4_2.transpose() + B2;
        Hess.template block<3, 3>(6, 3) = Hess.template block<3, 3>(3, 6).transpose();
        Hess.template block<3, 3>(3, 9) = M4_02_2 - N2_4;
        Hess.template block<3, 3>(9, 3) = Hess.template block<3, 3>(3, 9).transpose();

        Hess.template block<3, 3>(6, 6) =
            M1_1_1 + M1_1_1.transpose() - B1 + M2_2_2 + M2_2_2.transpose() - B2;
        Hess.template block<3, 3>(6, 9) = M2_02_2 - N2_2;
        Hess.template block<3, 3>(9, 6) = Hess.template block<3, 3>(6, 9).transpose();

        Hess.template block<3, 3>(9, 9) = -(N2_02 + N2_02.transpose());
    }
}  // namespace detail

/**
 * @brief Compute the gradient of the dihedral angle between two planes defined by four points.
 * 
 * Mid-Edge: V1-V2
 * Opposite Points: V0, V3
 */
template <typename T>
UIPC_GENERIC void dihedral_angle_gradient(const Eigen::Matrix<T, 3, 1>& v0,
                                          const Eigen::Matrix<T, 3, 1>& v1,
                                          const Eigen::Matrix<T, 3, 1>& v2,
                                          const Eigen::Matrix<T, 3, 1>& v3,
                                          Eigen::Matrix<T, 12, 1>&      grad)
{
    detail::dihedral_angle_gradient(v0, v1, v2, v3, grad);
}


template <typename T>
UIPC_GENERIC void dihedral_angle_hessian(const Eigen::Matrix<T, 3, 1>& v0,
                                         const Eigen::Matrix<T, 3, 1>& v1,
                                         const Eigen::Matrix<T, 3, 1>& v2,
                                         const Eigen::Matrix<T, 3, 1>& v3,
                                         Eigen::Matrix<T, 12, 12>&     Hess)
{
    detail::dihedral_angle_hessian(v0, v1, v2, v3, Hess);
}
}  // namespace uipc::backend::cuda
