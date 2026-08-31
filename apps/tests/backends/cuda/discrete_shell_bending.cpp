#include <app/app.h>
#include <finite_element/constitutions/discrete_shell_bending_reference.h>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
Float reference_metric(Float scale)
{
    const Vector3 x0{0.0, scale, 0.0};
    const Vector3 x1{0.0, 0.0, 0.0};
    const Vector3 x2{scale, 0.0, 0.0};
    const Vector3 x3{scale, -scale, 0.0};

    Float L0           = 0.0;
    Float h_bar        = 0.0;
    Float theta_bar    = 0.0;
    Float outer_weight = 0.0;
    detail::compute_discrete_shell_bending_reference(
        L0, h_bar, theta_bar, outer_weight, x0, x1, x2, x3);

    CHECK(L0 > 0.0);
    CHECK(h_bar > 0.0);
    CHECK(outer_weight == Catch::Approx(1.0));
    return outer_weight * L0 / h_bar;
}
}  // namespace

TEST_CASE("discrete_shell_bending_reference_weight", "[cuda][discrete_shell_bending]")
{
    const Float unit_metric   = reference_metric(1.0);
    const Float scaled_metric = reference_metric(2.5);

    CHECK(unit_metric == Catch::Approx(3.0));
    CHECK(scaled_metric == Catch::Approx(unit_metric));
}
