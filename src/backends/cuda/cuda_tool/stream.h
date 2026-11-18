/*
 * NVIDIA contributions: the SideStream fork/join helper.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <cuda_runtime.h>
// Provide a global-scope ::arg(std::complex) BEFORE any Eigen include. nvcc's
// passes cannot resolve Eigen MathFunctions.h's `using ::arg` otherwise. This must
// precede <uipc/common/type_define.h> (which pulls in <Eigen/Core>).
#if defined(__CUDACC__) && !defined(UIPC_EIGEN_ARG_SHIM)
#define UIPC_EIGEN_ARG_SHIM
#include <complex>
template <typename T>
__host__ __device__ inline T arg(const std::complex<T>& z)
{
    return std::atan2(std::imag(z), std::real(z));
}
#endif
#include <uipc/common/type_define.h>
#include <stdexcept>
#include <string>
#include <cstdio>
#include <span>

namespace uipc::backend::cuda_tool
{
using std::span;

// host-side error string without pulling in fmt (avoids nvcc/UTF-8 issues)
inline std::string cuda_error_string(cudaError_t e, const char* expr, const char* file, int line)
{
    char buf[512];
    std::snprintf(buf,
                  sizeof(buf),
                  "[%s:%d] CUDA error %s (%d): %s",
                  file,
                  line,
                  expr,
                  static_cast<int>(e),
                  cudaGetErrorString(e));
    return buf;
}
}  // namespace uipc::backend::cuda_tool

namespace uipc::backend::cuda_tool
{
// Default stream used by all cuda_tool primitives when no stream is given.
// Kept identical to muda's default (the default stream) so behavior is unchanged.
inline cudaStream_t default_stream()
{
    return cudaStream_t{nullptr};
}

inline void check_cuda_error(cudaError_t e, const char* expr, const char* file, int line)
{
    if(e != cudaSuccess)
    {
        throw std::runtime_error{cuda_error_string(e, expr, file, line)};
    }
}

#define CUDA_TOOL_CHECK(EXPR)                                                  \
    ::uipc::backend::cuda_tool::check_cuda_error((EXPR), #EXPR, __FILE__, __LINE__)

// Block size used by cuda_tool launches (matches muda's default_parallel_dim).
inline constexpr int default_block_dim = 256;

// Lightweight stream handle wrapper (replacement for cuda_tool::Stream).
// The default stream is a process-wide singleton identical to cudaStream_t{nullptr}.
class Stream
{
  public:
    Stream() = default;
    explicit Stream(cudaStream_t h) noexcept
        : m_handle(h)
    {
    }
    operator cudaStream_t() const noexcept { return m_handle; }
    cudaStream_t view() const noexcept { return m_handle; }

    static Stream& Default() noexcept
    {
        static Stream s{nullptr};
        return s;
    }

  private:
    cudaStream_t m_handle = nullptr;
};

// Owning non-blocking stream for work that runs beside another stream. It
// does not synchronize implicitly with the legacy default stream: fork()
// orders it after the work enqueued on the parent so far, join() orders the
// parent after the work enqueued on it so far. Everything it reads or writes
// must be enqueued between the two.
class SideStream
{
  public:
    SideStream() = default;
    ~SideStream()
    {
        if(m_join)
            cudaEventDestroy(m_join);
        if(m_fork)
            cudaEventDestroy(m_fork);
        if(m_stream)
            cudaStreamDestroy(m_stream);
    }
    SideStream(const SideStream&)            = delete;
    SideStream& operator=(const SideStream&) = delete;

    cudaStream_t fork(cudaStream_t parent = default_stream())
    {
        if(!m_stream)
        {
            CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&m_stream, cudaStreamNonBlocking));
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_fork, cudaEventDisableTiming));
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_join, cudaEventDisableTiming));
        }
        CUDA_TOOL_CHECK(cudaEventRecord(m_fork, parent));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(m_stream, m_fork, 0));
        return m_stream;
    }

    void join(cudaStream_t parent = default_stream())
    {
        CUDA_TOOL_CHECK(cudaEventRecord(m_join, m_stream));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(parent, m_join, 0));
    }

  private:
    cudaStream_t m_stream = nullptr;
    cudaEvent_t  m_fork   = nullptr;
    cudaEvent_t  m_join   = nullptr;
};
}  // namespace uipc::backend::cuda_tool
