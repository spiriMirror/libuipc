#pragma once
// Kernel-side string logging collected to the host after a launch.
// Replacement for muda::Logger / LoggerViewer / KernelCout.
#include <cuda_tool/stream.h>
#include <cuda_tool/buffer.h>
#include <vector>
#include <string>
#include <cstring>

namespace uipc::backend::cuda_tool
{
namespace details
{
    // A device-side ring of fixed-size log records. Kernels write via LoggerViewer;
    // the host drains it after synchronization.
    struct LogRecord
    {
        int  thread_id;
        char text[256];
    };
}  // namespace details

// Device-capturable handle to the log buffer. Pass by value into kernels.
class LoggerViewer
{
  public:
    LoggerViewer() = default;
    __host__ LoggerViewer(details::LogRecord* records, int* count, int capacity)
        : m_records(records)
        , m_count(count)
        , m_capacity(capacity)
    {
    }

    // CUDA device code has no snprintf; use the built-in device printf directly.
    // Records are emitted to the kernel's stdout (drained on the next sync).
    template <typename... Args>
    __device__ void operator()(const char* fmt, Args&&... args) const
    {
        printf(fmt, std::forward<Args>(args)...);
    }

  private:
    details::LogRecord* m_records  = nullptr;
    int*                m_count    = nullptr;
    int                 m_capacity = 0;
};

// Host-side owner of the log buffer. Drain with fetch().
class Logger
{
  public:
    explicit Logger(int capacity = 4096)
        : m_capacity(capacity)
    {
        m_records.resize(capacity);
        m_count.resize(1);
        reset();
    }

    LoggerViewer viewer()
    {
        return LoggerViewer{m_records.data(), m_count.data(), m_capacity};
    }
    LoggerViewer viewer() const
    {
        return LoggerViewer{const_cast<details::LogRecord*>(m_records.data()),
                            const_cast<int*>(m_count.data()),
                            m_capacity};
    }

    void reset(cudaStream_t s = default_stream())
    {
        CUDA_TOOL_CHECK(cudaMemsetAsync(m_count.data(), 0, sizeof(int), s));
    }

    // drain collected records into host strings and reset the device count.
    std::vector<std::string> fetch(cudaStream_t s = default_stream())
    {
        int n = 0;
        m_count.copy_to(&n, s);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        if(n > m_capacity)
            n = m_capacity;
        std::vector<details::LogRecord> host(n);
        if(n > 0)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(host.data(),
                                            m_records.data(),
                                            n * sizeof(details::LogRecord),
                                            cudaMemcpyDeviceToHost,
                                            s));
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        std::vector<std::string> out;
        out.reserve(n);
        for(auto& r : host)
            out.emplace_back(r.text);
        reset(s);
        return out;
    }

  private:
    int                           m_capacity;
    DeviceVector<details::LogRecord> m_records;
    DeviceVector<int>             m_count;
};

// A thread-local kernel console used across the backend (replaces KernelCout).
class KernelCout
{
  public:
    KernelCout() = default;

    static LoggerViewer viewer() { return instance().m_logger.viewer(); }

    static std::vector<std::string> fetch(cudaStream_t s = default_stream())
    {
        return instance().m_logger.fetch(s);
    }

  private:
    static KernelCout& instance()
    {
        thread_local static KernelCout* s_instance = new KernelCout();
        return *s_instance;
    }
    Logger m_logger;
};
}  // namespace uipc::backend::cuda_tool
