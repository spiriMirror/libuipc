#include <app/require_log.h>

namespace uipc::test
{
void CaptureSink::sink_it_(const spdlog::details::log_msg& msg)
{
    LogMsg log_msg;
    log_msg.level   = msg.level;
    log_msg.payload = fmt::format("{}\n", msg.payload);
    m_msg.push_back(log_msg);
}

std::shared_ptr<CaptureSink> CaptureSink::instance()
{
    static auto sink = std::make_shared<CaptureSink>();
    return sink;
}

std::shared_ptr<spdlog::logger> CaptureSink::test_logger()
{
    static auto logger = std::make_shared<spdlog::logger>("test_logger", instance());
    return logger;
}
}  // namespace uipc
