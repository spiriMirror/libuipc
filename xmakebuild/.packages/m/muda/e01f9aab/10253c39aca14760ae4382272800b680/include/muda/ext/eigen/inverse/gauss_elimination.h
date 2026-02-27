#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace muda::eigen
{
struct GaussEliminationInverse
{
    template <typename T, int N>
    MUDA_INLINE MUDA_GENERIC Eigen::Matrix<T, N, N> operator()(const Eigen::Matrix<T, N, N>& input)
    {
        Eigen::Matrix<T, N, N> result;

        constexpr auto eps = std::numeric_limits<T>::epsilon();
        constexpr int  dim = N;
        T              mat[dim][dim * 2];
        for(int i = 0; i < dim; i++)
        {
            for(int j = 0; j < 2 * dim; j++)
            {
                if(j < dim)
                {
                    mat[i][j] = input(i, j);  //[i, j];
                }
                else
                {
                    mat[i][j] = j - dim == i ? 1 : 0;
                }
            }
        }

        for(int i = 0; i < dim; i++)
        {
            if(abs(mat[i][i]) < eps)
            {
                int j;
                for(j = i + 1; j < dim; j++)
                {
                    if(abs(mat[j][i]) > eps)
                        break;
                }
                if(j == dim)
                    return result;
                for(int r = i; r < 2 * dim; r++)
                {
                    mat[i][r] += mat[j][r];
                }
            }
            T ep = mat[i][i];
            for(int r = i; r < 2 * dim; r++)
            {
                mat[i][r] /= ep;
            }

            for(int j = i + 1; j < dim; j++)
            {
                T e = -1 * (mat[j][i] / mat[i][i]);
                for(int r = i; r < 2 * dim; r++)
                {
                    mat[j][r] += e * mat[i][r];
                }
            }
        }

        for(int i = dim - 1; i >= 0; i--)
        {
            for(int j = i - 1; j >= 0; j--)
            {
                T e = -1 * (mat[j][i] / mat[i][i]);
                for(int r = i; r < 2 * dim; r++)
                {
                    mat[j][r] += e * mat[i][r];
                }
            }
        }


        for(int i = 0; i < dim; i++)
        {
            for(int r = dim; r < 2 * dim; r++)
            {
                result(i, r - dim) = mat[i][r];
            }
        }

        return result;
    }
};
}  // namespace muda::eigen
