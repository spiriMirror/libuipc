#pragma once
#include <finite_element/matrix_utils.h>
namespace uipc::backend::cuda::fem
{
UIPC_GENERIC Float invariant2(const Matrix3x3& F);
UIPC_GENERIC Float invariant2(const Vector3& Sigma);
UIPC_GENERIC Float invariant3(const Matrix3x3& F);
UIPC_GENERIC Float invariant3(const Vector3& Sigma);
UIPC_GENERIC Float invariant4(const Matrix3x3& F, const Vector3& a);
UIPC_GENERIC Float invariant5(const Matrix3x3& F, const Vector3& a);

//tex: $\frac{\partial \det(\mathbf{F})}{\partial F}$
UIPC_GENERIC Matrix3x3 dJdF(const Matrix3x3& F);

// inverse material coordinates
UIPC_GENERIC Matrix3x3 Dm_inv(const Vector3& X0,
                              const Vector3& X1,
                              const Vector3& X2,
                              const Vector3& X3);
UIPC_GENERIC Matrix3x3 Ds(const Vector3& X0, const Vector3& X1, const Vector3& X2, const Vector3& X3);
UIPC_GENERIC Matrix9x12 dFdx(const Matrix3x3& DmInv);

UIPC_GENERIC UIPC_INLINE Matrix<Float, 3, 4> tetrahedron_shape_gradients(const Matrix3x3& DmInv)
{
    Matrix<Float, 3, 4> gradients;
#pragma unroll
    for(int i = 0; i < 3; ++i)
    {
        gradients(i, 1) = DmInv(0, i);
        gradients(i, 2) = DmInv(1, i);
        gradients(i, 3) = DmInv(2, i);
        gradients(i, 0) = -gradients(i, 1) - gradients(i, 2) - gradients(i, 3);
    }
    return gradients;
}

UIPC_GENERIC UIPC_INLINE Vector3 project_F_gradient(const Vector3& shape_gradient,
                                                    const Vector9& dEdF)
{
    Vector3 result = Vector3::Zero();
#pragma unroll
    for(int i = 0; i < 3; ++i)
    {
#pragma unroll
        for(int j = 0; j < 3; ++j)
            result(i) += shape_gradient(j) * dEdF(3 * j + i);
    }
    return result;
}

UIPC_GENERIC UIPC_INLINE Matrix3x3 project_F_hessian_block(const Vector3& left_shape_gradient,
                                                           const Matrix9x9& ddEddF,
                                                           const Vector3& right_shape_gradient)
{
    Matrix3x3 result;
#pragma unroll
    for(int i = 0; i < 3; ++i)
    {
#pragma unroll
        for(int j = 0; j < 3; ++j)
        {
            Float value = 0.0;
#pragma unroll
            for(int k = 0; k < 3; ++k)
            {
#pragma unroll
                for(int l = 0; l < 3; ++l)
                {
                    value += left_shape_gradient(k) * ddEddF(3 * k + i, 3 * l + j)
                             * right_shape_gradient(l);
                }
            }
            result(i, j) = value;
        }
    }
    return result;
}

UIPC_GENERIC Matrix3x3 F(const Vector3&   x0,
                         const Vector3&   x1,
                         const Vector3&   x2,
                         const Vector3&   x3,
                         const Matrix3x3& DmInv);
}  // namespace uipc::backend::cuda::fem
