#pragma once
#include <spdlog/common.h>
#include <string_view>
#include <uipc/common/dllexport.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/span.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

namespace uipc
{
class UIPC_CORE_API Logger
{
  public:
    using Level = spdlog::level::level_enum;

    static Logger create_console_logger(std::string_view logger_name,
                                        spdlog::sink_ptr sink_ptr = nullptr);

    void set_level(spdlog::level::level_enum level);

    spdlog::level::level_enum get_level() const;

    void set_pattern(std::string_view pattern);

    template <typename... Args>
    void debug(std::string_view fmt, Args&&... args)
    {
        _debug(fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }

    template <typename... Args>
    void info(std::string_view fmt, Args&&... args)
    {
        _info(fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }

    template <typename... Args>
    void warn(std::string_view fmt, Args&&... args)
    {
        _warn(fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }

    template <typename... Args>
    void error(std::string_view fmt, Args&&... args)
    {
        _error(fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }

    template <typename... Args>
    void critical(std::string_view fmt, Args&&... args)
    {
        _critical(fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }

    template <typename... Args>
    void log(Level level, std::string_view fmt, Args&&... args)
    {
        _log(level, fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }

    static void   current_logger(Logger logger);
    static Logger current_logger();

  private:
    void _debug(std::string_view msg);
    void _info(std::string_view msg);
    void _warn(std::string_view msg);
    void _error(std::string_view msg);
    void _critical(std::string_view msg);
    void _log(spdlog::level::level_enum level, std::string_view msg);

    class Impl;
    S<Impl> m_impl;
};
}  // namespace uipc

// short-cut
namespace uipc::logger
{
UIPC_CORE_API void set_level(Logger::Level level);

Logger::Level get_level();

UIPC_CORE_API void set_pattern(std::string_view pattern);

template <typename... Args>
inline void debug(std::string_view fmt, Args&&... args)
{
    Logger::current_logger().debug(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(std::string_view fmt, Args&&... args)
{
    Logger::current_logger().info(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(std::string_view fmt, Args&&... args)
{
    Logger::current_logger().warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(std::string_view fmt, Args&&... args)
{
    Logger::current_logger().error(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void critical(std::string_view fmt, Args&&... args)
{
    Logger::current_logger().critical(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void log(Logger::Level level, std::string_view fmt, Args&&... args)
{
    Logger::current_logger().log(level, fmt, std::forward<Args>(args)...);
}
}  // namespace uipc::logger