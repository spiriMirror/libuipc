#include <spdlog/common.h>
#include <uipc/common/logger.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace uipc
{
class Logger::Impl
{
  public:
    Impl(std::string_view logger_name = "uipc", spdlog::sink_ptr sink_ptr = nullptr)
    {
        if(sink_ptr == nullptr)
        {
            m_logger = spdlog::stdout_color_mt(std::string(logger_name));
        }
        else
        {
            m_logger = std::make_shared<spdlog::logger>(std::string(logger_name), sink_ptr);
            spdlog::register_logger(m_logger);
        }
        m_logger->set_level(spdlog::level::info);
    }

    void log(spdlog::level::level_enum level, std::string_view msg)
    {
        m_logger->log(level, msg);
    }

    void set_level(spdlog::level::level_enum level)
    {
        m_logger->set_level(level);
    }

    void set_pattern(std::string_view pattern)
    {
        m_logger->set_pattern(std::string(pattern));
    }

    std::shared_ptr<spdlog::logger> m_logger;

    static Logger& current_logger_instance()
    {
        static Logger logger = Logger::create_console_logger();
        return logger;
    }
};


void Logger::set_level(spdlog::level::level_enum level)
{
    Impl::current_logger_instance().m_impl->set_level(level);
}

void Logger::set_pattern(std::string_view pattern)
{
    Impl::current_logger_instance().m_impl->set_pattern(pattern);
}

Logger Logger::create_console_logger(std::string_view logger_name, spdlog::sink_ptr sink_ptr)
{
    Logger logger;
    logger.m_impl = uipc::make_shared<Impl>(logger_name, sink_ptr);
    return logger;
}

void Logger::current_logger(Logger logger)
{
    Impl::current_logger_instance() = logger;
}

Logger Logger::current_logger()
{
    return Impl::current_logger_instance();
}

void Logger::_debug(std::string_view msg)
{
    m_impl->log(spdlog::level::debug, msg);
}

void Logger::_info(std::string_view msg)
{
    m_impl->log(spdlog::level::info, msg);
}

void Logger::_warn(std::string_view msg)
{
    m_impl->log(spdlog::level::warn, msg);
}

void Logger::_error(std::string_view msg)
{
    m_impl->log(spdlog::level::err, msg);
}

void Logger::_critical(std::string_view msg)
{
    m_impl->log(spdlog::level::critical, msg);
}

void Logger::_log(spdlog::level::level_enum level, std::string_view msg)
{
    m_impl->log(level, msg);
}
}  // namespace uipc
