#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <Eigen/Eigenvalues>
namespace muda
{
namespace eigen
{
    template <typename T, int N>
    MUDA_GENERIC void evd(const Eigen::Matrix<T, N, N>& M,
                          Eigen::Vector<T, N>&          eigen_values,
                          Eigen::Matrix<T, N, N>&       eigen_vectors)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, N, N>> eigen_solver;
        // NOTE:
        //  On CUDA, if N <= 3, compute() is not supported.
        //  So, we use computeDirect() instead.
        if constexpr(N <= 3)
            eigen_solver.computeDirect(M);
        else
            eigen_solver.compute(M);
        eigen_values  = eigen_solver.eigenvalues();
        eigen_vectors = eigen_solver.eigenvectors();
    }

    namespace details
    {
        template <typename T, int N>
        MUDA_GENERIC void find_maxValue_diagOff(const Eigen::Matrix<T, N, N>& M,
                                                int&                          p,
                                                int&                          q,
                                                T& max_value)
        {
            max_value = -1;
            for(int i = 0; i < N; ++i)
            {
                for(int j = i + 1; j < N; ++j)
                {
                    if(abs(M(i, j)) > max_value)
                    {
                        max_value = abs(M(i, j));
                        p         = i;
                        q         = j;
                    }
                }
            }
        }

        template <typename T, int N>
        MUDA_GENERIC T calc_sumDiagOff(const Eigen::Matrix<T, N, N>& M)
        {
            T sum = 0.0f;
            for(int i = 0; i < N; ++i)
            {
                for(int j = i + 1; j < N; ++j)
                {
                    sum += abs(M(i, j));
                }
            }
            return sum;
        }

        template <typename T, int N>
        MUDA_GENERIC void sort_eigensystem_optimized(Eigen::Vector<T, N>& eigen_values,
                                                     Eigen::Matrix<T, N, N>& eigen_vectors,
                                                     bool ascending = false)
        {
            // create index array
            int indices[N];
            for(int i = 0; i < N; i++)
            {
                indices[i] = i;
            }

            // sort the indices
            for(int i = 0; i < N - 1; i++)
            {
                for(int j = 0; j < N - i - 1; j++)
                {
                    bool should_swap =
                        ascending ?
                            (eigen_values(indices[j]) > eigen_values(indices[j + 1])) :
                            (eigen_values(indices[j]) < eigen_values(indices[j + 1]));

                    if(should_swap)
                    {
                        int temp       = indices[j];
                        indices[j]     = indices[j + 1];
                        indices[j + 1] = temp;
                    }
                }
            }

            // reorder the data by the indices
            Eigen::Vector<T, N>    sorted_values;
            Eigen::Matrix<T, N, N> sorted_vectors;

            for(int i = 0; i < N; i++)
            {
                sorted_values(i)      = eigen_values(indices[i]);
                sorted_vectors.col(i) = eigen_vectors.col(indices[i]);
            }

            // copy back
            eigen_values  = sorted_values;
            eigen_vectors = sorted_vectors;
        }

        template <typename T, int N>
        MUDA_GENERIC void jacobi_rotate(Eigen::Matrix<T, N, N>& M,
                                        Eigen::Matrix<T, N, N>& E,
                                        int                     p,
                                        int                     q)
        {
            if(std::abs(M(p, q)) < 1e-12)
                return;
            T tau = (M(q, q) - M(p, p)) / (2.0 * M(p, q));
            T t;
            if(tau >= 0)
            {
                t = 1.0 / (tau + std::sqrt(1.0 + tau * tau));
            }
            else
            {
                t = -1.0 / (-tau + std::sqrt(1.0 + tau * tau));
            }
            T c = 1.0 / std::sqrt(1.0 + t * t);
            T s = t * c;

            // optimiza ratation
            for(int i = 0; i < N; i++)
            {
                if(i != p && i != q)
                {
                    T Mip   = M(i, p);
                    T Miq   = M(i, q);
                    M(p, i) = M(i, p) = c * Mip - s * Miq;
                    M(q, i) = M(i, q) = s * Mip + c * Miq;
                }

                // update eigen vector
                T Eip   = E(i, p);
                T Eiq   = E(i, q);
                E(i, p) = c * Eip - s * Eiq;
                E(i, q) = s * Eip + c * Eiq;
            }

            // update diagonal and pq
            T Mpp = M(p, p);
            T Mqq = M(q, q);
            T Mpq = M(p, q);

            M(p, p) = c * c * Mpp + s * s * Mqq - 2 * c * s * Mpq;
            M(q, q) = s * s * Mpp + c * c * Mqq + 2 * c * s * Mpq;
            M(p, q) = M(q, p) = 0;
        }
    }  // namespace details

    /**
     * @brief calculate the Eigen System of a symmetric matrix
     */
    template <typename T, int N>
    MUDA_GENERIC void evd_jacobi(const Eigen::Matrix<T, N, N>& M,
                                 Eigen::Vector<T, N>&          eigen_values,
                                 Eigen::Matrix<T, N, N>&       eigen_vectors)
    {
        auto symmetrix = M;
        eigen_vectors.setIdentity();
        while(details::calc_sumDiagOff(symmetrix) > 1e-6)
        {
            int q = -1, p = -1;
            T   max_value;
            details::find_maxValue_diagOff(symmetrix, p, q, max_value);
            details::jacobi_rotate(symmetrix, eigen_vectors, p, q);
        }
        eigen_values = symmetrix.diagonal();
        details::sort_eigensystem_optimized(eigen_values, eigen_vectors, true);
    }
}  // namespace eigen
}  // namespace muda