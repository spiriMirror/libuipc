#include <type_define.h>  // Eigen in CUDA
#include <app/app.h>
#include <utils/distance/distance_flagged.h>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace test_ee_distance_flag
{
template <typename T>
T robust_segment_segment_distance2(const Eigen::Vector<T, 3>& ea0,
                                   const Eigen::Vector<T, 3>& ea1,
                                   const Eigen::Vector<T, 3>& eb0,
                                   const Eigen::Vector<T, 3>& eb1)
{
    T d00, d01, d10, d11;
    distance::point_point_distance2(ea0, eb0, d00);
    distance::point_point_distance2(ea0, eb1, d01);
    distance::point_point_distance2(ea1, eb0, d10);
    distance::point_point_distance2(ea1, eb1, d11);
    return std::min(std::min(d00, d01), std::min(d10, d11));
}

void check_case(const Vector3& ea0,
                const Vector3& ea1,
                const Vector3& eb0,
                const Vector3& eb1,
                Float          expected_dist2)
{
    auto flag = distance::edge_edge_distance_flag(ea0, ea1, eb0, eb1);
    auto dim  = distance::detail::active_count(flag);

    Float D_flagged;
    distance::edge_edge_distance2(flag, ea0, ea1, eb0, eb1, D_flagged);

    Float D_robust = robust_segment_segment_distance2(ea0, ea1, eb0, eb1);
    Float D_line;
    distance::edge_edge_distance2(ea0, ea1, eb0, eb1, D_line);

    // Near-parallel/collinear separated segments should not stay in EE line-line mode.
    CHECK(dim == 2);
    CHECK(D_flagged == Catch::Approx(expected_dist2).margin(1e-12));
    CHECK(D_robust == Catch::Approx(expected_dist2).margin(1e-12));
    CHECK(D_line < 1e-12);
}
}  // namespace test_ee_distance_flag

TEST_CASE("ee_distance_flag_parallel_collinear", "[distance][ee][cuda]")
{
    using namespace test_ee_distance_flag;

    // Printed from Noodles/main_single_nogui.py runtime case.
    SECTION("EE_35_36_37_38")
    {
        check_case(Vector3(-9.048450e-14, 1.363250e-01, -6.164740e-12),
                   Vector3(2.016403e-13, 1.403250e-01, -5.310797e-12),
                   Vector3(2.024192e-12, 1.443250e-01, -1.499360e-12),
                   Vector3(3.119794e-12, 1.483250e-01, 2.146554e-13),
                   1.6e-5);
    }

    SECTION("EE_34_35_36_37")
    {
        check_case(Vector3(-9.260670e-15, 1.323250e-01, -6.043656e-12),
                   Vector3(-9.048450e-14, 1.363250e-01, -6.164740e-12),
                   Vector3(2.016403e-13, 1.403250e-01, -5.310797e-12),
                   Vector3(2.024192e-12, 1.443250e-01, -1.499360e-12),
                   1.6e-5);
    }
}

TEST_CASE("point_edge_flag_degenerate_edge", "[distance][pe][cuda]")
{
    // Degenerate edge (e0 == e1) should fall back to a valid PP flag.
    const Vector3 p(2.0, 0.0, 0.0);
    const Vector3 e0(1.0, 0.0, 0.0);
    const Vector3 e1(1.0, 0.0, 0.0);

    auto flag = distance::point_edge_distance_flag(p, e0, e1);
    CHECK(distance::detail::active_count(flag) == 2);

    Float D = -1.0;
    distance::point_edge_distance2(flag, p, e0, e1, D);
    CHECK(D == Catch::Approx(1.0).margin(1e-12));
}
