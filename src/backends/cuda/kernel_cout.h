#pragma once
#include <cuda_tool/logger.h>

/*****************************************************************/ /**
* \file   kernel_cout.h
* \brief  Kernel-side console output for debugging.
*
* To use `cout << xxx` in a kernel:
*
* @code
* my_kernel<<<grid, block>>>(cout_view, ...);
* // in kernel: cout << "xxx";
* @endcode
*
* where `cout_view` is obtained from `KernelCout::viewer()`. Output goes
* through device printf and is flushed to stdout on the next sync.
*
* \author Lenovo
* \date   April 2025
*********************************************************************/

namespace uipc::backend::cuda
{
// Facade kept for call-site compatibility; the implementation lives in
// cuda_tool::KernelCout (device-printf based).
class KernelCout
{
  public:
    static cuda_tool::LoggerViewer viewer()
    {
        return cuda_tool::KernelCout::viewer();
    }
};
}  // namespace uipc::backend::cuda
