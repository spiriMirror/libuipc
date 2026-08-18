#pragma once
// Device-side small-matrix math used by constitutive models.
// Raw implementations; no muda dependency. Float is `double` in this project, so
// the heavy paths (evd/inverse/atomic_add) are written for that.
// The ::arg(std::complex) nvcc shim lives in stream.h (included first).
#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif
#include <cuda_tool/stream.h>
#include <Eigen/Core>
#include <Eigen/Geometry>  // cross()
#include <Eigen/LU>       // determinant()
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
    namespace details
    {
        template <typename T, int N>
        __host__ __device__ inline T sum_off_diag(const Eigen::Matrix<T, N, N>& M)
        {
            T sum = 0;
            for(int i = 0; i < N; ++i)
                for(int j = i + 1; j < N; ++j)
                    sum += M(i, j) * M(i, j);
            return sum;
        }

        template <typename T, int N>
        __host__ __device__ inline void
        find_max_off_diag(const Eigen::Matrix<T, N, N>& M, int& p, int& q, T& max_value)
        {
            p = 0;
            q = 1;
            max_value = M(p, q) > T(0) ? M(p, q) : -M(p, q);
            for(int i = 0; i < N; ++i)
                for(int j = i + 1; j < N; ++j)
                {
                    T v = M(i, j) > T(0) ? M(i, j) : -M(i, j);
                    if(v > max_value)
                    {
                        max_value = v;
                        p = i;
                        q = j;
                    }
                }
        }

        template <typename T, int N>
        __host__ __device__ inline void jacobi_rotate(Eigen::Matrix<T, N, N>& M,
                                                      Eigen::Matrix<T, N, N>& V,
                                                      int p, int q)
        {
            if(M(p, q) == T(0))
                return;
            T theta = (M(q, q) - M(p, p)) / (T(2) * M(p, q));
            T t = T(1) / (std::abs(theta) + std::sqrt(theta * theta + T(1)));
            if(theta < T(0))
                t = -t;
            T c = T(1) / std::sqrt(t * t + T(1));
            T s = t * c;
            // rotate columns p,q of V
            for(int k = 0; k < N; ++k)
            {
                T vkp     = V(k, p);
                T vkq     = V(k, q);
                V(k, p)   = c * vkp - s * vkq;
                V(k, q)   = s * vkp + c * vkq;
            }
            // similarity transform M <- G^T M G (G rotation on p,q)
            for(int k = 0; k < N; ++k)
            {
                T mkp   = M(k, p);
                T mkq   = M(k, q);
                M(k, p) = c * mkp - s * mkq;
                M(k, q) = s * mkp + c * mkq;
            }
            for(int k = 0; k < N; ++k)
            {
                T mpk   = M(p, k);
                T mqk   = M(q, k);
                M(p, k) = c * mpk - s * mqk;
                M(q, k) = s * mpk + c * mqk;
            }
            M(p, q) = M(q, p) = T(0);
        }

        template <typename T, int N>
        __host__ __device__ inline void
        sort_eigensystem(Eigen::Vector<T, N>& w, Eigen::Matrix<T, N, N>& V)
        {
            // ascending sort of eigenvalues, columns of V follow
            for(int i = 0; i < N - 1; ++i)
                for(int j = 0; j < N - 1 - i; ++j)
                    if(w(j) > w(j + 1))
                    {
                        std::swap(w(j), w(j + 1));
                        V.col(j).swap(V.col(j + 1));
                    }
        }
    }  // namespace details

    // ------------------------------------------------------------------
    // Symmetric eigen-decomposition via cyclic Jacobi. Works for any N
    // (project uses N in {2,3,6,9,12}). A = Q * diag(w) * Q^T, ascending.
    // ------------------------------------------------------------------
    template <typename T, int N>
    __host__ __device__ inline void evd(const Eigen::Matrix<T, N, N>& M,
                                        Eigen::Vector<T, N>&          eigen_values,
                                        Eigen::Matrix<T, N, N>&       eigen_vectors)
    {
        Eigen::Matrix<T, N, N> A = M;
        eigen_vectors.setIdentity();
        const T tol = std::numeric_limits<T>::epsilon() * T(1e3);
        int     iter = 0;
        const int max_iter = 100 * N;
        while(details::sum_off_diag(A) > tol && iter < max_iter)
        {
            int p, q;
            T   max_value;
            details::find_max_off_diag(A, p, q, max_value);
            details::jacobi_rotate(A, eigen_vectors, p, q);
            ++iter;
        }
        eigen_values = A.diagonal();
        details::sort_eigensystem(eigen_values, eigen_vectors);
    }

    // eigenvalues only
    template <typename T, int N>
    __host__ __device__ inline Eigen::Vector<T, N>
    evd(const Eigen::Matrix<T, N, N>& M)
    {
        Eigen::Vector<T, N>    w;
        Eigen::Matrix<T, N, N> Q;
        evd(M, w, Q);
        return w;
    }

    // ------------------------------------------------------------------
    // Matrix inverse. Analytic for 2x2/3x3, Gauss-Jordan with partial
    // pivoting otherwise (project uses N=12). Returns by value.
    // ------------------------------------------------------------------
    template <typename T, int N>
    __host__ __device__ inline Eigen::Matrix<T, N, N>
    inverse(const Eigen::Matrix<T, N, N>& m)
    {
        Eigen::Matrix<T, N, N>  A   = m;
        Eigen::Matrix<T, N, N>  inv = Eigen::Matrix<T, N, N>::Identity();
        for(int col = 0; col < N; ++col)
        {
            // partial pivot
            int piv = col;
            T   best = A(col, col) > T(0) ? A(col, col) : -A(col, col);
            for(int r = col + 1; r < N; ++r)
            {
                T v = A(r, col) > T(0) ? A(r, col) : -A(r, col);
                if(v > best)
                {
                    best = v;
                    piv = r;
                }
            }
            if(piv != col)
            {
                A.row(col).swap(A.row(piv));
                inv.row(col).swap(inv.row(piv));
            }
            T d = A(col, col);
            // normalize pivot row
            for(int j = 0; j < N; ++j)
            {
                A(col, j) /= d;
                inv(col, j) /= d;
            }
            // eliminate other rows
            for(int r = 0; r < N; ++r)
            {
                if(r == col)
                    continue;
                T f = A(r, col);
                if(f != T(0))
                {
                    for(int j = 0; j < N; ++j)
                    {
                        A(r, j) -= f * A(col, j);
                        inv(r, j) -= f * inv(col, j);
                    }
                }
            }
        }
        return inv;
    }

    // ------------------------------------------------------------------
    // SVD of a general 3x3 matrix: A = U * diag(s) * V^T (descending s).
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
        evd(AtA, w, V);
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
        U.col(2) = U.col(0).cross(U.col(1));
    }

    // polar decomposition A = R * S (R rotation, S symmetric PSD).
    // Name `pd` matches muda::eigen::pd used by the backend.
    template <typename Scalar>
    __host__ __device__ inline void pd(const Eigen::Matrix<Scalar, 3, 3>& A,
                                       Eigen::Matrix<Scalar, 3, 3>&       R,
                                       Eigen::Matrix<Scalar, 3, 3>&       S)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        Mat3 U, V;
        Vec3 s;
        svd(A, U, s, V);
        R = U * V.transpose();
        if(R.determinant() < Scalar(0))
        {
            U.col(2) = -U.col(2);
            s(2)     = -s(2);
            R        = U * V.transpose();
        }
        S = V * s.asDiagonal() * V.transpose();
    }
    // alias
    template <typename Scalar>
    __host__ __device__ inline void polar(const Eigen::Matrix<Scalar, 3, 3>& A,
                                          Eigen::Matrix<Scalar, 3, 3>&       R,
                                          Eigen::Matrix<Scalar, 3, 3>&       S)
    {
        pd(A, R, S);
    }

    // ------------------------------------------------------------------
    // Element-wise atomicAdd for Eigen matrices/vectors; returns the value
    // BEFORE the add (mirrors muda::eigen::atomic_add). Device-only.
    // ------------------------------------------------------------------
    template <typename T, int M, int N>
    __device__ inline Eigen::Matrix<T, M, N>
    atomic_add(Eigen::Matrix<T, M, N>& dst, const Eigen::Matrix<T, M, N>& src)
    {
        Eigen::Matrix<T, M, N> old;
        for(int i = 0; i < M; ++i)
            for(int j = 0; j < N; ++j)
                old(i, j) = atomicAdd(&dst(i, j), src(i, j));
        return old;
    }
}  // namespace eigen
}  // namespace uipc::backend::cuda_tool
