#include <app/app.h>
#include <contact_system/contact_models/codim_ipc_contact_function.h>
#include <utils/codim_thickness.h>
#include <cmath>
#include <limits>

using namespace uipc;
using namespace uipc::backend::cuda;
namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
namespace barrier = sym::codim_ipc_contact;

struct BarrierInput
{
    Float D;
    Float d_hat;
    Float thickness;
};

struct BarrierOutput
{
    Float value[3];
    Float active_expression[3];
};

UIPC_GENERIC BarrierOutput evaluate_barrier(BarrierInput input)
{
    constexpr Float kappa     = 1e9;
    const auto [D, d_hat, xi] = input;
    BarrierOutput result;
    barrier::KappaBarrier(result.value[0], kappa, D, d_hat, xi);
    barrier::dKappaBarrierdD(result.value[1], kappa, D, d_hat, xi);
    barrier::ddKappaBarrierddD(result.value[2], kappa, D, d_hat, xi);

    // The pre-existing active expressions must remain unchanged below the bound.
    if(xi == 0)
    {
        barrier::KappaBarrierLog2(result.active_expression[0], kappa, D, d_hat);
        barrier::dKappaBarrierLog2dD(result.active_expression[1], kappa, D, d_hat);
        barrier::ddKappaBarrierLog2ddD(result.active_expression[2], kappa, D, d_hat);
    }
    else
    {
        barrier::KappaBarrierLog2WithThickness(result.active_expression[0], kappa, D, d_hat, xi);
        barrier::dKappaBarrierLog2WithThicknessdD(
            result.active_expression[1], kappa, D, d_hat, xi);
        barrier::ddKappaBarrierLog2WithThicknessddD(
            result.active_expression[2], kappa, D, d_hat, xi);
    }
    return result;
}

__global__ void evaluate_barriers_kernel(cuda_tool::CBufferView<BarrierInput> inputs,
                                         cuda_tool::BufferView<BarrierOutput> outputs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= inputs.size())
        return;
    outputs(i) = evaluate_barrier(inputs(i));
}
}  // namespace

TEST_CASE("ipc_barrier_upper_bound", "[cuda][contact][barrier]")
{
    std::vector<BarrierInput> inputs;
    for(Float thickness : {0.0, 1e-6, 1e-3})
    {
        for(Float d_hat : {5e-5, 0.01, 0.5})
        {
            const Vector2 range = D_range(thickness, d_hat);
            const Float   lower = range(0), upper = range(1);
            // Explicit adjacent floating-point values make the boundary checks
            // independent of compiler-specific geometry/distance rounding.
            for(Float D : {lower + 0.25 * (upper - lower),
                           lower + 0.75 * (upper - lower),
                           std::nextafter(upper, lower),
                           upper,
                           std::nextafter(upper, std::numeric_limits<Float>::infinity()),
                           upper + 0.5 * (upper - lower)})
                inputs.push_back({D, d_hat, thickness});
        }
    }

    cuda_tool::DeviceBuffer<BarrierInput>  device_inputs;
    cuda_tool::DeviceBuffer<BarrierOutput> device_outputs;
    device_inputs.copy_from(inputs);
    device_outputs.resize(inputs.size());
    constexpr int block_size = 128;
    evaluate_barriers_kernel<<<(inputs.size() + block_size - 1) / block_size, block_size>>>(
        device_inputs.cview(), device_outputs.view());
    std::vector<BarrierOutput> device_results;
    device_outputs.copy_to(device_results);
    REQUIRE(device_results.size() == inputs.size());

    for(SizeT i = 0; i < inputs.size(); ++i)
    {
        const auto [D, d_hat, thickness] = inputs[i];
        const Vector2       range        = D_range(thickness, d_hat);
        const BarrierOutput host_result  = evaluate_barrier(inputs[i]);
        for(bool device : {false, true})
        {
            const auto& result = device ? device_results[i] : host_result;
            CAPTURE(device, D, d_hat, thickness);
            for(int derivative = 0; derivative < 3; ++derivative)
            {
                CAPTURE(derivative);
                const Float value = result.value[derivative];
                REQUIRE(std::isfinite(value));
                if(D >= range(1))
                    CHECK(value == 0);
                else
                    CHECK(value
                          == Catch::Approx(result.active_expression[derivative]).epsilon(1e-12));
            }
            if(D < range(0) + 0.9 * (range(1) - range(0)))
            {
                CHECK(result.value[0] > 0);
                CHECK(result.value[1] < 0);
                CHECK(result.value[2] > 0);
            }
        }
    }
}
