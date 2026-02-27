#pragma once
#include <muda/muda_def.h>
#include <muda/check/check_cuda_errors.h>
#include <cinttypes>
#include <muda/literal/unit.h>
#include <muda/logger/logger_viewer.h>
#include <muda/buffer/device_var.h>
#include <vector>
#include <muda/tools/temp_buffer.h>

namespace muda
{
class LoggerMetaData
{
  public:
    uint32_t        id;
    LoggerBasicType type;
    void*           data;
    LoggerFmtArg    fmt_arg;
    template <typename T>
    const T& as();
};

class LoggerDataContainer
{
  public:
    span<LoggerMetaData> meta_data() { return m_meta_data; }

  private:
    friend class Logger;
    std::vector<LoggerMetaData> m_meta_data;
    std::vector<char>           m_buffer;
};

class Logger
{
    static constexpr size_t DEFAULT_META_SIZE   = 16_M;
    static constexpr size_t DEFAULT_BUFFER_SIZE = 128_M;

  public:
    Logger(LoggerViewer* global_viewer,
           size_t        meta_size   = DEFAULT_META_SIZE,
           size_t        buffer_size = DEFAULT_BUFFER_SIZE);

    Logger(size_t meta_size = DEFAULT_META_SIZE, size_t buffer_size = DEFAULT_BUFFER_SIZE)
        : Logger(nullptr, meta_size, buffer_size)
    {
    }

    ~Logger();

    // delete copy
    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;

    // allow move
    Logger(Logger&&) noexcept;
    Logger& operator=(Logger&&) noexcept;


    void retrieve(std::ostream& o = std::cout);

    MUDA_NODISCARD LoggerDataContainer retrieve_meta();

    MUDA_NODISCARD bool is_meta_data_full() const
    {
        return m_h_offset.exceed_meta_data;
    }

    MUDA_NODISCARD bool is_buffer_full() const
    {
        return m_h_offset.exceed_buffer;
    }

    MUDA_NODISCARD LoggerViewer viewer() const
    {
        return m_log_viewer_ptr ? *m_log_viewer_ptr : m_viewer;
    }

  private:
    friend class LaunchCore;
    friend class Debug;
    void expand_meta_data();
    void expand_buffer();
    void upload();
    void download();
    void expand_if_needed();

    //details::LoggerMetaData* m_meta_data;
    //size_t                   m_meta_data_size;


    details::TempBuffer<uint32_t>                m_sorted_meta_data_id;
    details::TempBuffer<details::LoggerMetaData> m_sorted_meta_data;

    details::TempBuffer<uint32_t>                m_meta_data_id;
    details::TempBuffer<details::LoggerMetaData> m_meta_data;

    std::vector<details::LoggerMetaData> m_h_meta_data;

    //char*              m_buffer;
    //size_t             m_buffer_size;
    details::TempBuffer<char> m_buffer;
    std::vector<char>         m_h_buffer;

    //details::LoggerOffset*           m_offset;
    details::TempBuffer<details::LoggerOffset> m_offset;
    details::LoggerOffset                      m_h_offset;

    LoggerViewer* m_log_viewer_ptr = nullptr;
    LoggerViewer  m_viewer;
    template <typename F>
    void _retrieve(F&&);
    void put(std::ostream& os, const details::LoggerMetaData& meta_data) const;
};
//MUDA_INLINE __device__ LoggerViewer cout;
}  // namespace muda

#include <muda/logger/details/logger.inl>