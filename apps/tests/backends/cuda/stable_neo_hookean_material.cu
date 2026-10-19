#include <app/app.h>
#include <finite_element/constitutions/stable_neo_hookean_3d_function.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/geometry/utils/factory.h>

using namespace uipc;
using namespace uipc::backend::cuda;

TEST_CASE("stable_neo_hookean_physical_calibration", "[cuda][stable_neo_hookean]")
{
    const vector<Vector3> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const vector<Vector4i>         cells = {{0, 1, 2, 3}};
    auto                           mesh  = geometry::tetmesh(vertices, cells);
    constitution::StableNeoHookean material;

    for(Float young : {1.0e3, 1.0e5, 1.2e5})
    {
        for(Float poisson : {-0.2, 0.0, 0.3, 0.49, 0.499})
        {
            CAPTURE(young, poisson);
            material.apply_to(mesh, constitution::ElasticModuli::youngs_poisson(young, poisson));
            const Float mu = mesh.tetrahedra().find<Float>("mu")->view().front();
            const Float lambda = mesh.tetrahedra().find<Float>("lambda")->view().front();
            const Float physical_mu = young / (2 * (1 + poisson));
            const Float physical_lambda =
                young * poisson / ((1 + poisson) * (1 - 2 * poisson));
            const Matrix3x3 identity = Matrix3x3::Identity();
            Matrix3x3       P;
            snk1::dEdF(P, mu, lambda, identity);
            CHECK(P.norm() < young * 1e-12);

            // Each component of the actual stress derivative must match Hooke's
            // law: dP = mu_L*(dF+dF^T) + lambda_L*trace(dF)*I.
            Matrix9x9 tangent;
            snk1::ddEddF_spd(tangent, mu, lambda, identity);
            for(int column = 0; column < 9; ++column)
            {
                Matrix3x3 direction      = Matrix3x3::Zero();
                direction.data()[column] = 1;
                const Matrix3x3 expected =
                    physical_mu * (direction + direction.transpose())
                    + physical_lambda * direction.trace() * identity;
                constexpr Float step  = 1e-6;
                const Matrix3x3 plus  = identity + step * direction;
                const Matrix3x3 minus = identity - step * direction;
                Matrix3x3       Pplus, Pminus;
                snk1::dEdF(Pplus, mu, lambda, plus);
                snk1::dEdF(Pminus, mu, lambda, minus);
                const Matrix3x3 measured = (Pplus - Pminus) / (2 * step);
                CHECK((measured - expected).norm() < expected.norm() * 1e-7);
                const Eigen::Map<const Vector9> expected_column(expected.data());
                CHECK((tangent.col(column) - expected_column).norm() < expected.norm() * 1e-7);
            }
        }
    }
}
