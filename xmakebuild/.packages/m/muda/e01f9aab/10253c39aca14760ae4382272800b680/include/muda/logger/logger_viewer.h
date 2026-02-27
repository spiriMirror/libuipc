#pragma once
#include <muda/logger/logger_basic_data.h>
#include <muda/muda_def.h>
#include <muda/check/check_cuda_errors.h>
#include <muda/literal/unit.h>
#include <muda/viewer/dense.h>
namespace muda
{
class LoggerViewer;
class LogProxy
{
    LoggerViewer* m_viewer = nullptr;
    uint32_t      m_log_id;

  public:
    MUDA_DEVICE LogProxy() = default;
    MUDA_DEVICE LogProxy(LoggerViewer& viewer);

    MUDA_DEVICE LogProxy(const LogProxy& other)
        : m_viewer(other.m_viewer)
        , m_log_id(other.m_log_id)
    {
    }

    template <bool IsFmt>
    MUDA_DEVICE LogProxy& push_string(const char* str);

    MUDA_DEVICE LogProxy& operator<<(const char* str);

#define PROXY_OPERATOR(enum_name, T)                                           \
    MUDA_INLINE MUDA_DEVICE friend LogProxy operator<<(LogProxy p, T v)        \
    {                                                                          \
        details::LoggerMetaData meta;                                          \
        meta.type = LoggerBasicType::enum_name;                                \
        meta.size = sizeof(T);                                                 \
        meta.id   = p.m_log_id;                                                \
        p.push_data(meta, &v);                                                 \
        return p;                                                              \
    }

    PROXY_OPERATOR(Int8, int8_t);
    PROXY_OPERATOR(Int16, int16_t);
    PROXY_OPERATOR(Int32, int32_t);
    PROXY_OPERATOR(Int64, int64_t);


    PROXY_OPERATOR(UInt8, uint8_t);
    PROXY_OPERATOR(UInt16, uint16_t);
    PROXY_OPERATOR(UInt32, uint32_t);
    PROXY_OPERATOR(UInt64, uint64_t);

    PROXY_OPERATOR(Float, float);
    PROXY_OPERATOR(Double, double);

#ifdef _WIN32
    PROXY_OPERATOR(Long, long);
    PROXY_OPERATOR(ULong, unsigned long);
#elif __linux__
    PROXY_OPERATOR(LongLong, long long);
    PROXY_OPERATOR(ULongLong, unsigned long long);
#endif

#undef PROXY_OPERATOR

    template <typename T>
    MUDA_DEVICE void push_fmt_arg(const T& obj, LoggerFmtArg fmt_arg_func);

    MUDA_DEVICE bool push_data(const details::LoggerMetaData& meta, const void* data);
};
class LoggerViewer
{
  public:
    friend class Logger;
    friend class LogProxy;

    template <typename T>
    MUDA_DEVICE LogProxy operator<<(const T& t);
    MUDA_DEVICE LogProxy operator<<(const char* s);
    template <bool IsFmt>
    MUDA_DEVICE LogProxy push_string(const char* str);
    MUDA_DEVICE LogProxy proxy() { return LogProxy(*this); }

    MUDA_GENERIC operator bool() const
    {
        return m_meta_data && m_buffer && m_offset;
    }

  private:
    // Don't use viewer, cuda don't allow to use constructor in __device__ global variable
    // However, LoggerViewer should be able to use as a global variable for debugging
    uint32_t*                m_meta_data_id      = nullptr;
    int                      m_meta_data_id_size = 0;
    details::LoggerMetaData* m_meta_data         = nullptr;
    int                      m_meta_data_size    = 0;
    char*                    m_buffer            = nullptr;
    int                      m_buffer_size       = 0;
    details::LoggerOffset*   m_offset            = nullptr;

    MUDA_DEVICE uint32_t next_meta_data_idx() const;
    MUDA_DEVICE uint32_t next_buffer_idx(uint32_t size) const;
    MUDA_DEVICE bool push_data(details::LoggerMetaData meta, const void* data);
};
}  // namespace muda

#include <muda/logger/details/logger_viewer.inl>