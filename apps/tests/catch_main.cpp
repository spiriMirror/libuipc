#include <catch2/catch_all.hpp>
#include <uipc/common/logger.h>

#include <algorithm>
#include <string>

namespace
{
uipc::Logger::Level parse_log_level(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });

    if(value == "trace")
        return spdlog::level::trace;
    if(value == "debug")
        return spdlog::level::debug;
    if(value == "info")
        return spdlog::level::info;
    if(value == "warn" || value == "warning")
        return spdlog::level::warn;
    if(value == "error")
        return spdlog::level::err;
    if(value == "critical")
        return spdlog::level::critical;
    if(value == "off")
        return spdlog::level::off;

    // Keep default if unknown.
    return uipc::logger::get_level();
}
}  // namespace

int main(int argc, char* argv[])
{
    Catch::Session session;

    std::string log_level;
    auto cli = session.cli() | Catch::Clara::Opt(log_level, "level")["--log-level"](
                                   "Set logger level: trace|debug|info|warn|error|critical|off");

    session.cli(cli);

    const int result = session.applyCommandLine(argc, argv);
    if(result != 0)
        return result;

    if(!log_level.empty())
        uipc::logger::set_level(parse_log_level(log_level));

    return session.run();
}
