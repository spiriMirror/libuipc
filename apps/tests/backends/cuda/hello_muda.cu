#include <muda/ext/eigen/eigen_core_cxx20.h> // to use Eigen in CUDA

#include <app/app.h>
#include <muda/buffer/device_buffer.h>

void hello_muda()
{
    using namespace cuda_tool;

    DeviceBuffer<int> a(100);
    DeviceBuffer<int> b(100);
    DeviceBuffer<int> c(100);

    a.fill(1);
    b.fill(2);
    c.fill(0);

    std::vector<int> result(100, 0);
    std::vector<int> expected(100, 3);

    cuda_tool::ParallelFor()
        .kernel_name("hello_muda")
        .apply(a.size(),
               [a = a.cviewer(),
                b = b.cviewer(),
                c = c.viewer()] __device__(int i) mutable
               { c(i) = a(i) + b(i); });

    c.copy_to(result);

    REQUIRE(result == expected);
}

TEST_CASE("hello_muda", "[muda]")
{
    hello_muda();
}

