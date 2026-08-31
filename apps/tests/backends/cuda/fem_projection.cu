#include <finite_element/fem_utils.h>

#include <app/app.h>

namespace
{
using namespace uipc;
using namespace uipc::backend::cuda;

TEST_CASE("FEM tetrahedron projection matches dense dFdx", "[cuda][fem]")
{
    Matrix3x3 Dm_inv;
    Dm_inv << 0.7, -0.2, 0.4, 0.3, 1.1, -0.6, -0.5, 0.8, 1.3;

    Vector9 dEdF;
    dEdF << -0.3, 0.7, 1.1, -1.4, 0.2, 0.6, 0.9, -0.8, 1.7;

    Matrix9x9 ddEddF;
    for(int i = 0; i < 9; ++i)
    {
        for(int j = 0; j < 9; ++j)
            ddEddF(i, j) = 0.1 * (i + 1) * (j + 2) + (i == j ? 2.0 : 0.0);
    }
    ddEddF = 0.5 * (ddEddF + ddEddF.transpose()).eval();

    auto shape_gradients = fem::tetrahedron_shape_gradients(Dm_inv);
    Matrix<Float, 3, 4> expected_shape_gradients;
    expected_shape_gradients << -0.5, 0.7, 0.3, -0.5, -1.7, -0.2, 1.1, 0.8,
        -1.1, 0.4, -0.6, 1.3;
    REQUIRE(shape_gradients.isApprox(expected_shape_gradients, 1e-12));

    Matrix9x12 dFdx = Matrix9x12::Zero();
    for(int vertex = 0; vertex < 4; ++vertex)
    {
        for(int component = 0; component < 3; ++component)
        {
            for(int column = 0; column < 3; ++column)
            {
                dFdx(3 * column + component, 3 * vertex + component) =
                    expected_shape_gradients(column, vertex);
            }
        }
    }

    Vector12    dense_gradient = dFdx.transpose() * dEdF;
    Matrix12x12 dense_hessian  = dFdx.transpose() * ddEddF * dFdx;

    for(int i = 0; i < 4; ++i)
    {
        Vector3 projected_gradient =
            fem::project_F_gradient(shape_gradients.col(i), dEdF);
        REQUIRE(projected_gradient.isApprox(dense_gradient.segment<3>(3 * i), 1e-12));

        for(int j = 0; j < 4; ++j)
        {
            Matrix3x3 projected_hessian = fem::project_F_hessian_block(
                shape_gradients.col(i), ddEddF, shape_gradients.col(j));
            REQUIRE(projected_hessian.isApprox(dense_hessian.block<3, 3>(3 * i, 3 * j), 1e-12));
        }
    }
}
}  // namespace
