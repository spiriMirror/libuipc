#pragma once
#include <muda/logger.h>
#include <muda/ext/eigen/log_proxy.h>

/*****************************************************************/ /**
 * \file   kernel_cout.h
 * \brief  Remove the original implementation, because of NVCC unstablity
 * 
 * To use: `cout << xxx` in kernel, you should now
 * 
 * @code
 * 
 * ParallelFor()
 *     .apply(N, 
 * [cout = KernelCout::viewer()] __device__ (int i) mutable
 * { 
 *     cout << "xxx"; 
 * })
 * 
 * @endcode
 * 
 * \author Lenovo
 * \date   April 2025
 *********************************************************************/

namespace uipc::backend::cuda
{
class KernelCout
{
  public:
    static muda::LoggerViewer viewer();

  private:
    KernelCout();
    muda::LoggerViewer _viewer();
    std::stringstream  m_string_stream;
    muda::Logger       m_logger;
    void               init();
};
}  // namespace uipc::backend::cuda
