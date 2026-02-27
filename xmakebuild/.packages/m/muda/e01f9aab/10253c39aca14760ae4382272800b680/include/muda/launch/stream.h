#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include <muda/check/check_cuda_errors.h>
#include <muda/tools/temp_buffer.h>

namespace muda
{
template <typename T>
class DeviceBuffer;

/// <summary>
/// RAII wrapper for cudaStream
/// </summary>
class Stream
{
    cudaStream_t m_handle = nullptr;

  public:
    enum class Flag : unsigned int
    {
        eDefault     = cudaStreamDefault,
        eNonBlocking = cudaStreamNonBlocking
    };

    MUDA_NODISCARD Stream(Flag f = Flag::eDefault);
    ~Stream();

    operator cudaStream_t() const { return m_handle; }
    cudaStream_t view() const { return m_handle; }

    // delete copy constructor and copy assignment operator
    Stream(const Stream&)            = delete;
    Stream& operator=(const Stream&) = delete;

    // allow move constructor and move assignment operator
    Stream(Stream&& o) MUDA_NOEXCEPT;
    Stream& operator=(Stream&& o) MUDA_NOEXCEPT;

    void wait() const;

    void begin_capture(cudaStreamCaptureMode mode = cudaStreamCaptureModeThreadLocal) const;
    void end_capture(cudaGraph_t* graph) const;

    static Stream& Default();

    class TailLaunch
    {
      public:
        MUDA_DEVICE TailLaunch(){};
        MUDA_DEVICE operator cudaStream_t() const;
    };

    class FireAndForget
    {
      public:
        MUDA_DEVICE FireAndForget(){};
        MUDA_DEVICE operator cudaStream_t() const;
    };

    class GraphTailLaunch
    {
      public:
        MUDA_DEVICE GraphTailLaunch(){};
        MUDA_DEVICE operator cudaStream_t() const;
    };

    class GraphFireAndForget
    {
      public:
        MUDA_DEVICE GraphFireAndForget(){};
        MUDA_DEVICE operator cudaStream_t() const;
    };

    std::byte* workspace(size_t byte_size);

  private:
    Stream(nullptr_t)
        : m_handle(nullptr)
    {
    }
    details::ByteTempBuffer m_workspace;
};


}  // namespace muda

#include "details/stream.inl"