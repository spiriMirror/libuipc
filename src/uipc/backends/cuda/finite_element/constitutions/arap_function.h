#pragma once
#include <type_define.h>
#include <muda/ext/eigen/svd.h>

namespace uipc::backend::cuda
{
namespace sym::arap_3d
{
    constexpr double sqrt2 =
        1.4142135623730950488016887242096980785696718753769480731766797379907324784621070388503875343276;

    // Function to compute the ARAP energy
    template <typename T>
    __host__ __device__ void E(T&                            energy,
                               const T&                      kappa,
                               const T&                      v,
                               const Eigen::Matrix<T, 3, 3>& F)
    {
        Eigen::Matrix<T, 3, 3> R;
        Eigen::Matrix<T, 3, 3> U, V;
        Eigen::Vector3<T>      Sigma;
        muda::eigen::svd(F, U, Sigma, V);
        R      = U * V.transpose();
        energy = kappa * v * (F - R).squaredNorm();
    }

    // Function to compute the gradient of the ARAP energy
    template <typename T>
    __host__ __device__ void dEdF(Eigen::Vector<T, 9>&          gradient,
                                  const T&                      kappa,
                                  const T&                      v,
                                  const Eigen::Matrix<T, 3, 3>& F)
    {
        Eigen::Matrix<T, 3, 3> R;
        Eigen::Matrix<T, 3, 3> U, V;
        Eigen::Vector3<T>      Sigma;
        muda::eigen::svd(F, U, Sigma, V);
        R = U * V.transpose();

        Eigen::Matrix<T, 3, 3> dPsi_dF = 2 * (F - R);

        T kv = kappa * v;

        gradient.segment<3>(0) = kv * dPsi_dF.col(0);
        gradient.segment<3>(3) = kv * dPsi_dF.col(1);
        gradient.segment<3>(6) = kv * dPsi_dF.col(2);
    }

    // Function to flatten a matrix into a vector
    template <typename T>
    __host__ __device__ Eigen::Matrix<T, 9, 1> vec(const Eigen::Matrix<T, 3, 3>& mat)
    {
        Eigen::Matrix<T, 9, 1> vector;
        vector << mat(0, 0), mat(1, 0), mat(2, 0), mat(0, 1), mat(1, 1),
            mat(2, 1), mat(0, 2), mat(1, 2), mat(2, 2);
        return vector;
    }

    // Function to compute the ARAP Hessian
    template <typename T>
    __host__ __device__ void ARAP_Hessian(Eigen::Matrix<T, 9, 9>&       H,
                                          const Eigen::Matrix<T, 3, 3>& F)
    {
        Eigen::Matrix<T, 3, 3> U, V;
        Eigen::Vector3<T>      Sigma;
        muda::eigen::svd(F, U, Sigma, V);
        // Define the twist modes
        Eigen::Matrix<T, 3, 3> T0, T1, T2;
        T0.row(0) = Eigen::Vector3<T>(0, -1, 0);
        T0.row(1) = Eigen::Vector3<T>(1, 0, 0);
        T0.row(2) = Eigen::Vector3<T>(0, 0, 0);
        T1.row(0) = Eigen::Vector3<T>(0, 0, 0);
        T1.row(1) = Eigen::Vector3<T>(0, 0, 1);
        T1.row(2) = Eigen::Vector3<T>(0, -1, 0);
        T2.row(0) = Eigen::Vector3<T>(0, 0, 1);
        T2.row(1) = Eigen::Vector3<T>(0, 0, 0);
        T2.row(2) = Eigen::Vector3<T>(-1, 0, 0);

        T0 = (1 / sqrt2) * U * T0 * V.transpose();
        T1 = (1 / sqrt2) * U * T1 * V.transpose();
        T2 = (1 / sqrt2) * U * T2 * V.transpose();

        // Flatten the twist modes
        Eigen::Matrix<T, 9, 1> t0 = vec(T0);
        Eigen::Matrix<T, 9, 1> t1 = vec(T1);
        Eigen::Matrix<T, 9, 1> t2 = vec(T2);

        // Get the singular values
        T s0 = Sigma(0);
        T s1 = Sigma(1);
        T s2 = Sigma(2);

        // Compute the Hessian
        H.setIdentity();
        H *= 2;
        H -= (4 / (s0 + s1)) * (t0 * t0.transpose());
        H -= (4 / (s1 + s2)) * (t1 * t1.transpose());
        H -= (4 / (s0 + s2)) * (t2 * t2.transpose());
    }

    // Function to compute the Hessian of the ARAP energy
    template <typename T>
    __host__ __device__ void ddEddF(Eigen::Matrix<T, 9, 9>&       hessian,
                                    const T&                      kappa,
                                    const T&                      v,
                                    const Eigen::Matrix<T, 3, 3>& F)
    {
        ARAP_Hessian(hessian, F);
        hessian *= kappa * v;
    }
}  // namespace sym::arap_3d
}  // namespace uipc::backend::cuda
