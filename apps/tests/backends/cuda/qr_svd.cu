#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <algorithm/qr_svd.hpp>
#include <cmath>
#include <vector>

namespace
{
__global__ void wilkinson_shift_float_kernel(uipc::backend::cuda_tool::BufferView<float> result)
{
    namespace math = uipc::backend::cuda::math;
    result(0)      = math::wilkinson_shift(-0.0f, 1.0f, 0.0f);
    result(1)      = math::wilkinson_shift(2.0f, 1.0f, 0.0f);
    result(2)      = math::wilkinson_shift(0.0f, 1.0f, 2.0f);
}
}  // namespace

TEST_CASE("float Wilkinson shift is stable on GPU", "[cuda][qr_svd]")
{
    using uipc::backend::cuda_tool::DeviceBuffer;

    DeviceBuffer<float> device_result(3);
    wilkinson_shift_float_kernel<<<1, 1>>>(device_result.view());

    std::vector<float> result(3);
    device_result.copy_to(result);

    const float root_two = std::sqrt(2.0f);
    CHECK(result[0] == Catch::Approx(-1.0f));
    CHECK(result[1] == Catch::Approx(1.0f - root_two).margin(1e-6));
    CHECK(result[2] == Catch::Approx(1.0f + root_two).margin(1e-6));

    CHECK(uipc::backend::cuda::math::wilkinson_shift(-0.0f, 1.0f, 0.0f)
          == Catch::Approx(-1.0f));
}
