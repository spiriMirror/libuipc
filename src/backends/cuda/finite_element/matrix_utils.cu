#include <finite_element/matrix_utils.h>
#include <muda/ext/eigen/svd.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
UIPC_GENERIC Vector9 flatten(const Matrix3x3& A) noexcept
{
    Vector9 column;

    unsigned int index = 0;
    for(unsigned int j = 0; j < A.cols(); j++)
        for(unsigned int i = 0; i < A.rows(); i++, index++)
            column[index] = A(i, j);

    return column;
}

UIPC_GENERIC Matrix3x3 unflatten(const Vector9& v) noexcept
{
    Matrix3x3      A;
    unsigned int index = 0;
    for(unsigned int j = 0; j < A.cols(); j++)
        for(unsigned int i = 0; i < A.rows(); i++, index++)
            A(i, j) = v[index];

    return A;
}

UIPC_GENERIC Float ddot(const Matrix3x3& A, const Matrix3x3& B)
{
    Float result = 0;
    for(int y = 0; y < 3; y++)
        for(int x = 0; x < 3; x++)
            result += A(x, y) * B(x, y);

    return result;
}

UIPC_GENERIC void svd(const Matrix3x3& F, Matrix3x3& U, Vector3& Sigma, Matrix3x3& V) noexcept
{
    // this function is already tested in the muda eigen test
    muda::eigen::svd(F, U, Sigma, V);
}

UIPC_GENERIC void polar_decomposition(const Matrix3x3& F, Matrix3x3& R, Matrix3x3& S) noexcept
{
    // this function is already tested in the muda eigen test
    muda::eigen::pd(F, R, S);
}

UIPC_GENERIC void evd(const Matrix3x3& A, Vector3& eigen_values, Matrix3x3& eigen_vectors) noexcept
{
    // this function is already tested in the muda eigen test
    muda::eigen::evd(A, eigen_values, eigen_vectors);
}

UIPC_GENERIC void evd(const Matrix9x9& A, Vector9& eigen_values, Matrix9x9& eigen_vectors) noexcept
{
    // this function is already tested in the muda eigen test
    muda::eigen::evd(A, eigen_values, eigen_vectors);
}

UIPC_GENERIC void evd(const Matrix12x12& A, Vector12& eigen_values, Matrix12x12& eigen_vectors) noexcept
{
    muda::eigen::evd(A, eigen_values, eigen_vectors);
}

UIPC_GENERIC Matrix9x9 clamp_to_spd(const Matrix9x9& A) noexcept
{
    // clamp directly
    Matrix9x9 Q;
    Vector9 values;
    muda::eigen::evd(A, values, Q);
    for(int x = 0; x < 9; x++)
        values[x] = (values[x] > 0.0) ? values[x] : 0.0;
    Matrix9x9 B = Q * values.asDiagonal() * Q.transpose();
    return B;
}

UIPC_GENERIC Matrix12x12 clamp_to_spd(const Matrix12x12& A) noexcept
{
    // clamp directly
    Matrix12x12 Q;
    Vector12 values;
    muda::eigen::evd(A, values, Q);
    for(int x = 0; x < 12; x++)
        values[x] = (values[x] > 0.0) ? values[x] : 0.0;
    Matrix12x12 B = Q * values.asDiagonal() * Q.transpose();
    return B;
}
}  // namespace apba
