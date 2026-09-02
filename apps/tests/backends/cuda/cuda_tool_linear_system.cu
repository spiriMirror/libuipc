#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/linear_reduction.h>

#include <app/app.h>
#include <cmath>
#include <limits>

namespace
{
using uipc::backend::cuda_tool::DeviceDenseVector;
using uipc::backend::cuda_tool::LinearSystemContext;

template <typename T>
DeviceDenseVector<T> make_device_vector(const std::vector<T>& host)
{
    DeviceDenseVector<T> device;
    device.resize_discard(host.size());
    device.buffer_view().copy_from(host.data());
    return device;
}

TEST_CASE("cuda_tool CUB linear reductions", "[cuda][cuda_tool][linear_system]")
{
    LinearSystemContext context;

    SECTION("dot and norm support float vectors across capacity growth")
    {
        auto x = make_device_vector<float>({3.0f, 4.0f, 12.0f});
        auto y = make_device_vector<float>({2.0f, -1.0f, 0.5f});
        CHECK(context.dot(x, y) == Catch::Approx(8.0f));
        CHECK(context.norm(x) == Catch::Approx(13.0f));

        std::vector<float> grown_x(513, 2.0f);
        std::vector<float> grown_y(513, -0.25f);
        x = make_device_vector<float>(grown_x);
        y = make_device_vector<float>(grown_y);
        CHECK(context.dot(x, y) == Catch::Approx(-256.5f));
        CHECK(context.norm(x) == Catch::Approx(2.0 * std::sqrt(513.0)).epsilon(1e-5));

        x = make_device_vector<float>({6.0f, 8.0f});
        CHECK(context.norm(x) == Catch::Approx(10.0f));
    }

    SECTION("scaled double norm avoids overflow and underflow")
    {
        auto values = make_device_vector<double>({1.0e200, 3.0, 1.0e-200});
        const double norm = context.norm(values);
        CHECK(std::isfinite(norm));
        CHECK(norm == Catch::Approx(1.0e200).epsilon(1e-14));
    }

    SECTION("empty vectors and invalid dot shapes are explicit")
    {
        DeviceDenseVector<double> empty;
        CHECK(context.dot(empty, empty) == 0.0);
        CHECK(context.norm(empty) == 0.0);

        auto one = make_device_vector<double>({1.0});
        auto two = make_device_vector<double>({1.0, 2.0});
        CHECK_THROWS_AS(context.dot(one, two), std::invalid_argument);
    }

    SECTION("non-finite norm inputs remain diagnosable")
    {
        auto infinite =
            make_device_vector<double>({std::numeric_limits<double>::infinity(), 1.0});
        auto not_a_number =
            make_device_vector<double>({std::numeric_limits<double>::quiet_NaN(), 1.0});
        CHECK(std::isinf(context.norm(infinite)));
        CHECK(std::isnan(context.norm(not_a_number)));
    }
}
}  // namespace
