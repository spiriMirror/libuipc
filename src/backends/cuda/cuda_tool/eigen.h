#pragma once
// Device-side small-matrix math used by constitutive models.
// Raw implementations; no muda dependency.
#include <cuda_tool/stream.h>
// Match muda's device-side Eigen setup. nvcc's device pass cannot find ::arg for
// std::complex (Eigen MathFunctions.h does `using ::arg`), so provide one, exactly
// as muda does in ext/eigen/eigen_cxx20.h.
#if defined(__CUDACC__)
#include <complex>
// global-scope overload so Eigen's `using ::arg` resolves in both passes
template <typename T>
__host__ __device__ inline T arg(const std::complex<T>& z)
{
    return std::atan2(std::imag(z), std::real(z));
}
#endif
#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif
#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace uipc::backend::cuda_tool
{
using Matrix3f = Eigen::Matrix<float, 3, 3>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3f = Eigen::Vector3f;
using Vector3d = Eigen::Vector3d;

namespace eigen
{
    // ------------------------------------------------------------------
    // Symmetric eigen-decomposition of a 3x3 matrix via the analytic
    // (Smith) closed form, robust on device. Returns (eigenvalues asc,
    // eigenvectors as columns of Q), A = Q * diag(w) * Q^T.
    // ------------------------------------------------------------------
    template <typename Scalar>
    __host__ __device__ inline void sym_eig(const Eigen::Matrix<Scalar, 3, 3>& A,
                                            Eigen::Matrix<Scalar, 3, 1>& w,
                                            Eigen::Matrix<Scalar, 3, 3>& Q)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        const Scalar eps = std::numeric_limits<Scalar>::epsilon() * Scalar(10);

        // tridiagonal reduction would be overkill; use the direct cubic method.
        Scalar p1 = A(0, 1) * A(0, 1) + A(0, 2) * A(0, 2) + A(1, 2) * A(1, 2);
        if(p1 < eps)
        {
            w = Vec3(A(0, 0), A(1, 1), A(2, 2));
            Q.setIdentity();
            // sort ascending with simple permutation
            return;
        }
        Scalar q  = A.trace() / Scalar(3);
        Scalar p2 = (A(0, 0) - q) * (A(0, 0) - q) + (A(1, 1) - q) * (A(1, 1) - q)
                    + (A(2, 2) - q) * (A(2, 2) - q) + Scalar(2) * p1;
        Scalar p   = std::sqrt(p2 / Scalar(6));
        Mat3   B   = (A - q * Mat3::Identity()) / p;  // traceless, eigenvalues in [-2,2]
        Scalar r   = B.determinant() / Scalar(2);
        // clamp r to [-1,1] for acos stability
        if(r <= Scalar(-1))
            r = Scalar(-1);
        else if(r >= Scalar(1))
            r = Scalar(1);
        Scalar phi = std::acos(r) / Scalar(3);

        // eigenvalues, unsorted
        Scalar e1 = q + Scalar(2) * p * std::cos(phi);
        Scalar e3 = q + Scalar(2) * p * std::cos(phi + Scalar(2.09439510239319549));  // 2pi/3
        Scalar e2 = Scalar(3) * q - e1 - e3;

        w = Vec3(e1, e2, e3);

        // eigenvectors: null space of (A - w_i I)
        for(int k = 0; k < 3; ++k)
        {
            Mat3   M  = A - w(k) * Mat3::Identity();
            Vec3   r0 = M.row(0), r1 = M.row(1), r2 = M.row(2);
            Vec3   c01 = r0.cross(r1);
            Vec3   c12 = r1.cross(r2);
            Vec3   c20 = r2.cross(r0);
            Scalar n01 = c01.squaredNorm(), n12 = c12.squaredNorm(), n20 = c20.squaredNorm();
            Vec3   v;
            if(n01 >= n12 && n01 >= n20)
                v = c01;
            else if(n12 >= n20)
                v = c12;
            else
                v = c20;
            Scalar n = v.norm();
            if(n < eps)
                v = Vec3::Unit(k);  // degenerate: pick basis
            else
                v /= n;
            Q.col(k) = v;
        }
    }

    // eigenvalues only
    template <typename Scalar>
    __host__ __device__ inline Eigen::Matrix<Scalar, 3, 1>
    evd(const Eigen::Matrix<Scalar, 3, 3>& A)
    {
        Eigen::Matrix<Scalar, 3, 1> w;
        Eigen::Matrix<Scalar, 3, 3> Q;
        sym_eig(A, w, Q);
        return w;
    }
    // eigenvalues + eigenvectors
    template <typename Scalar>
    __host__ __device__ inline void evd(const Eigen::Matrix<Scalar, 3, 3>& A,
                                        Eigen::Matrix<Scalar, 3, 1>&       w,
                                        Eigen::Matrix<Scalar, 3, 3>&       Q)
    {
        sym_eig(A, w, Q);
    }

    // ------------------------------------------------------------------
    // SVD of a general 3x3 matrix: A = U * diag(s) * V^T
    // via eigendecomposition of A^T A.
    // ------------------------------------------------------------------
    template <typename Scalar>
    __host__ __device__ inline void svd(const Eigen::Matrix<Scalar, 3, 3>& A,
                                        Eigen::Matrix<Scalar, 3, 3>&       U,
                                        Eigen::Matrix<Scalar, 3, 1>&       s,
                                        Eigen::Matrix<Scalar, 3, 3>&       V)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        Mat3 AtA = A.transpose() * A;
        Vec3 w;
        sym_eig(AtA, w, V);
        // singular values = sqrt of eigenvalues (clamp negatives to 0)
        for(int k = 0; k < 3; ++k)
            s(k) = std::sqrt(w(k) < Scalar(0) ? Scalar(0) : w(k));
        // sort descending
        for(int i = 0; i < 2; ++i)
            for(int j = 0; j < 2 - i; ++j)
                if(s(j) < s(j + 1))
                {
                    std::swap(s(j), s(j + 1));
                    V.col(j).swap(V.col(j + 1));
                }
        // U columns = A * v_i / s_i
        const Scalar eps = std::numeric_limits<Scalar>::epsilon() * Scalar(100);
        for(int k = 0; k < 3; ++k)
        {
            Vec3 v  = V.col(k);
            Vec3 av = A * v;
            if(s(k) > eps)
                U.col(k) = av / s(k);
            else
                U.col(k) = Vec3::Unit(k);
        }
        // enforce orthonormality / right-handedness minimally
        U.col(2) = U.col(0).cross(U.col(1));
    }

    // polar decomposition A = R * S (R rotation, S symmetric PSD)
    template <typename Scalar>
    __host__ __device__ inline void polar(const Eigen::Matrix<Scalar, 3, 3>& A,
                                          Eigen::Matrix<Scalar, 3, 3>&       R,
                                          Eigen::Matrix<Scalar, 3, 3>&       S)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        Mat3 U, V;
        Vec3 s;
        svd(A, U, s, V);
        R = U * V.transpose();
        // correct reflection
        if(R.determinant() < Scalar(0))
        {
            U.col(2) = -U.col(2);
            s(2)     = -s(2);
            R        = U * V.transpose();
        }
        S = V * s.asDiagonal() * V.transpose();
    }

    // rotation part only
    template <typename Scalar>
    __host__ __device__ inline Eigen::Matrix<Scalar, 3, 3>
    polar_rotation(const Eigen::Matrix<Scalar, 3, 3>& A)
    {
        Eigen::Matrix<Scalar, 3, 3> R, S;
        polar(A, R, S);
        return R;
    }

    // ------------------------------------------------------------------
    // matrix logarithm of a symmetric PSD matrix (used by log_proxy)
    // ------------------------------------------------------------------
    template <typename Scalar>
    __host__ __device__ inline Eigen::Matrix<Scalar, 3, 3>
    logm_psd(const Eigen::Matrix<Scalar, 3, 3>& A)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        Vec3 w;
        Mat3 Q;
        sym_eig(A, w, Q);
        const Scalar eps = std::numeric_limits<Scalar>::epsilon() * Scalar(100);
        Vec3           lw;
        for(int k = 0; k < 3; ++k)
            lw(k) = std::log(w(k) > eps ? w(k) : eps);
        return Q * lw.asDiagonal() * Q.transpose();
    }
}  // namespace eigen
}  // namespace uipc::backend::cuda_tool
