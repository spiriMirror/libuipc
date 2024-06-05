#include <muda/logger.h>
#include <uipc/backends/cuda/kernel_cout.h>

namespace uipc::backend
{
__device__ muda::LoggerViewer cout;

muda::LoggerViewer*           device_logger_viewer_ptr() noexcept
{
    muda::LoggerViewer* ptr = nullptr;
    // In IDE this will show an error, but it's fine
    // don't change the `cout` to `&cout`
    // or you get runtime error
    checkCudaErrors(cudaGetSymbolAddress((void**)&ptr, cout));
    return ptr;
}
}  // namespace uipc::backend
