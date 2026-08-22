#include <cuda_tool/cuda_tool.h>  // cuda_tool smoke test: named kernel + raw <<<>>> launch

#include <app/app.h>

namespace
{
__global__ void hello_muda_kernel(uipc::backend::cuda_tool::CBufferView<int> a,
                                  uipc::backend::cuda_tool::CBufferView<int> b,
                                  uipc::backend::cuda_tool::BufferView<int>  c,
                                  int                                        n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    c(i) = a(i) + b(i);
}
}  // namespace

void hello_muda()
{
    using namespace uipc::backend::cuda_tool;

    DeviceBuffer<int> a(100);
    DeviceBuffer<int> b(100);
    DeviceBuffer<int> c(100);

    a.fill(1);
    b.fill(2);
    c.fill(0);

    std::vector<int> result(100, 0);
    std::vector<int> expected(100, 3);

    auto k = hello_muda_kernel;
    int  n = (int)a.size();
    if(n > 0)
    {
        k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
            a.cview(), b.cview(), c.view(), n);
    }

    c.copy_to(result);

    REQUIRE(result == expected);
}

TEST_CASE("hello_muda", "[muda]")
{
    hello_muda();
}
