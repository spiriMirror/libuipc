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
    static void set_level(spdlog::level::level_enum level);

    static Logger create_console_logger(std::string_view logger_name = "uipc",
                                        spdlog::sink_ptr sink_ptr    = nullptr);

    static void set_pattern(std::string_view pattern);

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

    template <typename ... Args>
    void log(spdlog::level::level_enum level, std::string_view fmt, Args&&... args)
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

namespace uipc::log
{
using spdlog::level::level_enum;
// short-cut
inline void set_level(spdlog::level::level_enum level)
{
    Logger::set_level(level);
}

inline void set_pattern(std::string_view pattern)
{
    Logger::set_pattern(pattern);
}

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

template <typename ... Args>
inline void log(spdlog::level::level_enum level, std::string_view fmt, Args&&... args)
{
    Logger::current_logger().log(level, fmt, std::forward<Args>(args)...);
}
}