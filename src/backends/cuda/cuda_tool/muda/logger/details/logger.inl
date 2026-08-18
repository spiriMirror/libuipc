#include <algorithm>
#include <sstream>
#include <muda/mstl/span.h>
#include <muda/cub/device/device_radix_sort.h>
namespace muda
{
template <typename T>
MUDA_INLINE const T& LoggerMetaData::as()
{
    if constexpr(std::is_same_v<T, int8_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::Int8, "");
    }
    else if constexpr(std::is_same_v<T, int16_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::Int16, "");
    }
    else if constexpr(std::is_same_v<T, int32_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::Int32, "");
    }
    else if constexpr(std::is_same_v<T, int64_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::Int64, "");
    }
    else if constexpr(std::is_same_v<T, uint8_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::UInt8, "");
    }
    else if constexpr(std::is_same_v<T, uint16_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::UInt16, "");
    }
    else if constexpr(std::is_same_v<T, uint32_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::UInt32, "");
    }
    else if constexpr(std::is_same_v<T, uint64_t>)
    {
        MUDA_ASSERT(type == LoggerBasicType::UInt64, "");
    }
    else if constexpr(std::is_same_v<T, float>)
    {
        MUDA_ASSERT(type == LoggerBasicType::Float, "");
    }
    else if constexpr(std::is_same_v<T, double>)
    {
        MUDA_ASSERT(type == LoggerBasicType::Double, "");
    }
    else
    {
        MUDA_ASSERT(type == LoggerBasicType::Object, "");
    };
    return *reinterpret_cast<const T*>(data);
}

MUDA_INLINE Logger::Logger(LoggerViewer* global_viewer, size_t meta_size, size_t buffer_size)
    : m_meta_data_id(meta_size)
    , m_meta_data(meta_size)
    , m_sorted_meta_data_id(meta_size)
    , m_sorted_meta_data(meta_size)
    , m_h_meta_data(meta_size)
    , m_buffer(buffer_size)
    , m_h_buffer(buffer_size)
    , m_log_viewer_ptr(global_viewer)
    , m_offset(1)
{
    upload();
}

MUDA_INLINE Logger::Logger(Logger&& other) noexcept
    : m_meta_data_id(std::move(other.m_meta_data_id))
    , m_meta_data(std::move(other.m_meta_data))
    , m_sorted_meta_data_id(std::move(other.m_sorted_meta_data_id))
    , m_sorted_meta_data(std::move(other.m_sorted_meta_data))
    , m_h_meta_data(std::move(other.m_h_meta_data))
    , m_buffer(std::move(other.m_buffer))
    , m_h_buffer(std::move(other.m_h_buffer))
    , m_offset(std::move(other.m_offset))
    , m_h_offset(std::move(other.m_h_offset))
    , m_log_viewer_ptr(std::move(other.m_log_viewer_ptr))
{
    other.m_log_viewer_ptr = nullptr;
    other.m_viewer         = {};
}

MUDA_INLINE Logger& Logger::operator=(Logger&& other) noexcept
{
    if(this == &other)
        return *this;

    m_meta_data_id         = std::move(other.m_meta_data_id);
    m_meta_data            = std::move(other.m_meta_data);
    m_sorted_meta_data_id  = std::move(other.m_sorted_meta_data_id);
    m_sorted_meta_data     = std::move(other.m_sorted_meta_data);
    m_h_meta_data          = std::move(other.m_h_meta_data);
    m_buffer               = std::move(other.m_buffer);
    m_h_buffer             = std::move(other.m_h_buffer);
    m_offset               = std::move(other.m_offset);
    m_h_offset             = std::move(other.m_h_offset);
    m_log_viewer_ptr       = std::move(other.m_log_viewer_ptr);
    other.m_log_viewer_ptr = nullptr;
    other.m_viewer         = {};

    return *this;
}
template <typename F>
void Logger::_retrieve(F&& f)
{
    // don't allow automatic sync in this region
    // or it may cause infinite loop
    auto is_debug_sync = muda::Debug::is_debug_sync_all();
    muda::Debug::debug_sync_all(false);

    download();
    //auto meta_data_span =
    //    span<details::LoggerMetaData>{m_h_meta_data}.subspan(0, m_h_offset.meta_data_offset);
    //std::stable_sort(meta_data_span.begin(),
    //                 meta_data_span.end(),
    //                 [](const details::LoggerMetaData& a, const details::LoggerMetaData& b)
    //                 { return a.id < b.id; });

    auto meta_data_span =
        span<details::LoggerMetaData>{m_h_meta_data}.subspan(0, m_h_offset.meta_data_offset);

    f(meta_data_span);

    expand_if_needed();
    upload();
    muda::Debug::debug_sync_all(is_debug_sync);
}
MUDA_INLINE void Logger::retrieve(std::ostream& os)
{
    std::stringstream ss;
    Logger::_retrieve(
        [&](const span<details::LoggerMetaData>& meta_data_span)
        {
            for(const auto& meta_data : meta_data_span)
            {
                if(meta_data.exceeded)
                    ss << "[log_id " << meta_data.id << ": buffer exceeded]";
                else
                    put(ss, meta_data);
            }
        });
    os << ss.str();
}

MUDA_INLINE LoggerDataContainer Logger::retrieve_meta()
{
    LoggerDataContainer ret;
    Logger::_retrieve(
        [&](const span<details::LoggerMetaData>& meta_data_span)
        {
            // copy buffer for safety
            ret.m_buffer = m_h_buffer;
            auto buffer  = ret.m_buffer.data();
            ret.m_meta_data.resize(meta_data_span.size());
            std::transform(meta_data_span.begin(),
                           meta_data_span.end(),
                           ret.m_meta_data.begin(),
                           [buffer](const details::LoggerMetaData& meta_data)
                           {
                               return LoggerMetaData{meta_data.id,
                                                     meta_data.type,
                                                     buffer + meta_data.offset,
                                                     meta_data.fmt_arg};
                           });
        });
    return ret;
}

MUDA_INLINE void Logger::expand_meta_data()
{
    auto new_size = m_meta_data.size() * 2;

    m_meta_data_id.resize(new_size);
    m_meta_data.resize(new_size);

    m_sorted_meta_data_id.resize(new_size);
    m_sorted_meta_data.resize(new_size);
}

MUDA_INLINE void Logger::expand_buffer()
{
    auto new_size = m_buffer.size() * 2;
    m_buffer.resize(new_size);
}

MUDA_INLINE void Logger::upload()
{
    // reset
    m_h_offset = {};
    m_offset   = {m_h_offset};

    m_viewer.m_offset            = m_offset.data();
    m_viewer.m_meta_data_id      = m_meta_data_id.data();
    m_viewer.m_meta_data_id_size = static_cast<int>(m_meta_data_id.size());
    m_viewer.m_meta_data         = m_meta_data.data();
    m_viewer.m_meta_data_size    = static_cast<int>(m_meta_data.size());
    m_viewer.m_buffer            = m_buffer.data();
    m_viewer.m_buffer_size       = static_cast<int>(m_buffer.size());

    if(m_log_viewer_ptr)
    {
        checkCudaErrors(cudaMemcpyAsync(
            m_log_viewer_ptr, &m_viewer, sizeof(m_viewer), cudaMemcpyHostToDevice, nullptr));
    }
    checkCudaErrors(cudaDeviceSynchronize());
}

MUDA_INLINE void Logger::download()
{
    // copy back
    std::vector<details::LoggerOffset> h_offset(1);
    m_offset.copy_to(h_offset);
    m_h_offset = h_offset[0];

    // sort meta data

    DeviceRadixSort().SortPairs(m_meta_data_id.data(),
                                m_sorted_meta_data_id.data(),
                                m_meta_data.data(),
                                m_sorted_meta_data.data(),
                                m_h_offset.meta_data_offset);

    if(m_h_offset.meta_data_offset > 0)
    {
        m_h_meta_data.resize(m_h_offset.meta_data_offset);
        checkCudaErrors(cudaMemcpyAsync(m_h_meta_data.data(),
                                        m_sorted_meta_data.data(),
                                        m_h_meta_data.size() * sizeof(details::LoggerMetaData),
                                        cudaMemcpyDeviceToHost));
    }

    if(m_h_offset.buffer_offset > 0)
    {
        m_h_buffer.resize(m_h_offset.buffer_offset);
        checkCudaErrors(cudaMemcpyAsync(
            m_h_buffer.data(), m_buffer.data(), m_h_offset.buffer_offset, cudaMemcpyDeviceToHost));
    }

    checkCudaErrors(cudaDeviceSynchronize());
}

MUDA_INLINE void Logger::expand_if_needed()
{
    if(m_h_offset.exceed_meta_data)
    {
        auto old_size = m_meta_data.size();
        expand_meta_data();
        auto new_size = m_meta_data.size();

        m_h_offset.exceed_meta_data = 0;
        MUDA_KERNEL_WARN_WITH_LOCATION(
            "Logger meta data buffer expanded %d => %d", old_size, new_size);
    }
    if(m_h_offset.exceed_buffer)
    {
        auto old_size = m_buffer.size();
        expand_buffer();
        auto new_size = m_buffer.size();

        m_h_offset.exceed_buffer = 0;
        MUDA_KERNEL_WARN_WITH_LOCATION("Logger buffer expanded %d => %d", old_size, new_size);
    }
}

MUDA_INLINE void Logger::put(std::ostream& os, const details::LoggerMetaData& meta_data) const
{
    auto buffer = m_h_buffer.data();
    auto offset = meta_data.offset;
    auto type   = meta_data.type;
#define MUDA_PUT_CASE(EnumT, T)                                                \
    case LoggerBasicType::EnumT:                                               \
        os << *reinterpret_cast<const T*>(buffer + offset);                    \
        break;

    switch(type)
    {
        case LoggerBasicType::String:
            os << buffer + offset;
            break;
            MUDA_PUT_CASE(Int8, int8_t);
            MUDA_PUT_CASE(Int16, int16_t);
            MUDA_PUT_CASE(Int32, int32_t);
            MUDA_PUT_CASE(Int64, int64_t);
            MUDA_PUT_CASE(UInt8, uint8_t);
            MUDA_PUT_CASE(UInt16, uint16_t);
            MUDA_PUT_CASE(UInt32, uint32_t);
            MUDA_PUT_CASE(UInt64, uint64_t);
            MUDA_PUT_CASE(Float, float);
            MUDA_PUT_CASE(Double, double);
        default:
            MUDA_ERROR_WITH_LOCATION("Unknown type");
            break;
    }
#undef MUDA_PUT_CASE
}

MUDA_INLINE Logger::~Logger() {}
}  // namespace muda