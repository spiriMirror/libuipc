#include <muda/atomic.h>

namespace muda
{
MUDA_INLINE MUDA_DEVICE LogProxy::LogProxy(LoggerViewer& viewer)
    : m_viewer(&viewer)
{
    MUDA_KERNEL_ASSERT(m_viewer->m_buffer && m_viewer->m_meta_data,
                       "LoggerViewer is not initialized");
    m_log_id = atomic_add(&(m_viewer->m_offset->log_id), 1u);
}
template <bool IsFmt>
MUDA_INLINE MUDA_DEVICE LogProxy& LogProxy::push_string(const char* str)
{
    auto strlen = [](const char* s)
    {
        size_t len = 0;
        while(*s++)
            ++len;
        return len;
    };
    auto size = strlen(str) + 1;

    details::LoggerMetaData meta;
    if constexpr(IsFmt)
    {
        meta.type = LoggerBasicType::FmtString;
    }
    else
    {
        meta.type = LoggerBasicType::String;
    }
    meta.size = size;
    meta.id   = m_log_id;
    m_viewer->push_data(meta, str);
    return *this;
}

template <typename T>
MUDA_DEVICE void LogProxy::push_fmt_arg(const T& obj, LoggerFmtArg func)
{
    details::LoggerMetaData meta;
    meta.type    = LoggerBasicType::Object;
    meta.size    = sizeof(T);
    meta.id      = m_log_id;
    meta.fmt_arg = func;
    m_viewer->push_data(meta, &obj);
}

MUDA_INLINE MUDA_DEVICE bool LogProxy::push_data(const details::LoggerMetaData& meta,
                                            const void*                    data)
{
    return m_viewer->push_data(meta, data);
}

MUDA_INLINE MUDA_DEVICE LogProxy& LogProxy::operator<<(const char* str)
{
    return push_string<false>(str);
}

template <typename T>
MUDA_INLINE MUDA_DEVICE LogProxy LoggerViewer::operator<<(const T& t)
{
    auto p = LogProxy(*this);
    p << t;
    return p;
}

template <bool IsFmt>
MUDA_INLINE MUDA_DEVICE LogProxy LoggerViewer::push_string(const char* str)
{
    auto p = LogProxy(*this);
    p.push_string<IsFmt>(str);
    return p;
}

MUDA_INLINE MUDA_DEVICE LogProxy LoggerViewer::operator<<(const char* s)
{
    auto p = LogProxy(*this);
    p << s;
    return p;
}

MUDA_INLINE MUDA_DEVICE uint32_t next_idx(uint32_t* data_offset, uint32_t size, uint32_t total_size)
{

    uint32_t old = *data_offset;
    if(old + size >= total_size)
        return ~0u;
    uint32_t assumed;
    do
    {
        assumed         = old;
        auto new_offset = old + size;
        old             = atomic_cas(data_offset, assumed, new_offset);
        if(old + size >= total_size)
        {
            old = ~0u;
            break;
        }
    } while(assumed != old);
    return old;
}

MUDA_INLINE MUDA_DEVICE uint32_t LoggerViewer::next_meta_data_idx() const
{
    auto idx = next_idx(&(m_offset->meta_data_offset), 1u, m_meta_data_size);
    if(idx == ~0u)
    {
        atomic_cas(&(m_offset->exceed_meta_data), 0u, 1u);
        return ~0u;
    }
    return idx;
}

MUDA_INLINE MUDA_DEVICE uint32_t LoggerViewer::next_buffer_idx(uint32_t size) const
{
    auto idx = next_idx(&(m_offset->buffer_offset), size, m_buffer_size);
    if(idx == ~0u)
    {
        atomic_cas(&(m_offset->exceed_buffer), 0u, 1u);
        return ~0u;
    }
    return idx;
}

MUDA_INLINE MUDA_DEVICE bool LoggerViewer::push_data(details::LoggerMetaData meta,
                                                     const void* data)
{
    auto meta_idx = next_meta_data_idx();
    if(meta_idx == ~0u)
    {
        MUDA_KERNEL_WARN_WITH_LOCATION(
            "LoggerViewer: meta data is exceeded, "
            "the content[id=%d] will be discarded.",
            meta.id);
        return false;
    }
    auto buffer_idx = next_buffer_idx(meta.size);
    if(buffer_idx == ~0u)
    {
        meta.exceeded = true;
        MUDA_KERNEL_WARN_WITH_LOCATION(
            "LoggerViewer: log buffer is exceeded, "
            "the content[id=%d] will be discarded.",
            meta.id);

        m_meta_data[meta_idx]    = meta;
        m_meta_data_id[meta_idx] = meta.id;
        return false;
    }
    meta.offset              = buffer_idx;
    m_meta_data[meta_idx]    = meta;
    m_meta_data_id[meta_idx] = meta.id;
    for(int i = 0; i < meta.size; ++i)
        m_buffer[buffer_idx + i] = reinterpret_cast<const char*>(data)[i];
    return true;
}
}  // namespace muda
