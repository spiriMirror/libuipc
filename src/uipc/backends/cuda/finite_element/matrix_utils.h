#pragma once
#include <type_define.h>
#include <muda/muda_def.h>

// ref: https://github.com/theodorekim/HOBAKv1/blob/main/src/util/MATRIX_UTIL.h

namespace uipc::backend::cuda
{
using Matrix9x12 = Matrix<Float, 9, 12>;

// flatten a matrix3x3 to a vector9 in a consistent way
UIPC_GENERIC Vector9 flatten(const Matrix3x3& A) noexcept;

// unflatten a vector9 to a matrix3x3 in a consistent way
UIPC_GENERIC Matrix3x3 unflatten(const Vector9& v) noexcept;

UIPC_GENERIC Float ddot(const Matrix3x3& A, const Matrix3x3& B);

// compute the singular value decomposition of a matrix3x3
// the U,V are already tested and modified to be a rotation matrices
UIPC_GENERIC void svd(const Matrix3x3& F, Matrix3x3& U, Vector3& Sigma, Matrix3x3& V) noexcept;

// compute the polar decomposition of a matrix3x3
UIPC_GENERIC void polar_decomposition(const Matrix3x3& F, Matrix3x3& R, Matrix3x3& S) noexcept;

UIPC_GENERIC void evd(const Matrix3x3& A, Vector3& eigen_values, Matrix3x3& eigen_vectors) noexcept;

UIPC_GENERIC void evd(const Matrix9x9& A, Vector9& eigen_values, Matrix9x9& eigen_vectors) noexcept;

UIPC_GENERIC void evd(const Matrix12x12& A, Vector12& eigen_values, Matrix12x12& eigen_vectors) noexcept;

// clamp the eigenvalues of a matrix9x9 to be semi-positive-definite
UIPC_GENERIC Matrix9x9 clamp_to_spd(const Matrix9x9& A) noexcept;

// clamp the eigenvalues of a matrix12x12 to be semi-positive-definite
UIPC_GENERIC Matrix12x12 clamp_to_spd(const Matrix12x12& A) noexcept;
}  // namespace uipc::backend::cuda