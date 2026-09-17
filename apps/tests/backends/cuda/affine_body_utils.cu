#include <affine_body/utils.h>
#include <app/catch2.h>
#include <cuda_tool/buffer.h>

namespace
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace cuda_tool = uipc::backend::cuda_tool;

__global__ void torque_to_F_kernel(cuda_tool::CVarView<Vector12> q,
                                   cuda_tool::VarView<Vector12>  force)
{
    if(blockIdx.x != 0 || threadIdx.x != 0)
        return;
    *force = torque_to_F(2.0, Vector3::UnitZ(), *q);
}
}  // namespace

TEST_CASE("ABD torque conversion evaluates inverse transpose safely", "[cuda][affine_body]")
{
    Matrix3x3 A        = Matrix3x3::Identity();
    Vector12  expected = Vector12::Zero();

    SECTION("identity")
    {
        expected(4) = -1.0;
        expected(6) = 1.0;
    }

    SECTION("non-orthogonal affine matrix")
    {
        A << 2.0, 1.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0;
        // A^{-T} = [[1/2, 0, 0], [-1/6, 1/3, 0], [0, 0, 1/4]].
        expected(3) = 1.0 / 6.0;
        expected(4) = -1.0 / 3.0;
        expected(6) = 1.0 / 2.0;
    }

    SECTION("singular affine matrix still drops torque")
    {
        A.row(1) = A.row(0);
    }

    Vector12 q = Vector12::Zero();
    for(int row = 0; row < 3; ++row)
        q.segment<3>(3 + 3 * row) = A.row(row);

    const Vector12 host_force = torque_to_F(2.0, Vector3::UnitZ(), q);

    cuda_tool::DeviceVar<Vector12> device_q{q};
    cuda_tool::DeviceVar<Vector12> device_force;
    torque_to_F_kernel<<<1, 1>>>(device_q.cview(), device_force.view());
    CUDA_TOOL_CHECK(cudaGetLastError());
    const Vector12 gpu_force = device_force;

    for(int i = 0; i < 12; ++i)
    {
        CAPTURE(i);
        CHECK(host_force(i) == Catch::Approx(expected(i)).margin(1e-12));
        CHECK(gpu_force(i) == Catch::Approx(expected(i)).margin(1e-12));
    }
}
