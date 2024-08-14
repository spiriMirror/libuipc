#pragma once
#include <finite_element/matrix_utils.h>
namespace uipc::backend::cuda::fem
{
MUDA_GENERIC Float invariant2(const Matrix3x3& F);
MUDA_GENERIC Float invariant2(const Vector3& Sigma);
MUDA_GENERIC Float invariant3(const Matrix3x3& F);
MUDA_GENERIC Float invariant3(const Vector3& Sigma);
MUDA_GENERIC Float invariant4(const Matrix3x3& F, const Vector3& a);
MUDA_GENERIC Float invariant5(const Matrix3x3& F, const Vector3& a);

//tex: $\frac{\partial \det(\mathbf{F})}{\partial F}$
MUDA_GENERIC Matrix3x3 dJdF(const Matrix3x3& F);

// inverse material coordinates
MUDA_GENERIC Matrix3x3 Dm_inv(const Vector3& X0, const Vector3& X1, const Vector3& X2, const Vector3& X3);
MUDA_GENERIC Matrix3x3 Ds(const Vector3& X0, const Vector3& X1, const Vector3& X2, const Vector3& X3);
MUDA_GENERIC Matrix9x12 dFdx(const Matrix3x3& DmInv);
MUDA_GENERIC Matrix3x3    F(const Vector3& x0,
                          const Vector3& x1,
                          const Vector3& x2,
                          const Vector3& x3,
                          const Matrix3x3& DmInv);

MUDA_GENERIC void EG_to_lame(Float E, Float G, Float& lambda, Float& mu, Float& poisson);
MUDA_GENERIC void lame_to_EG(Float lambda, Float mu, Float& E, Float& G, Float& poisson);
}  // namespace uipc::backend::cuda