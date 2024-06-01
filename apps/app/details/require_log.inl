#include <algorithm>
#include <fmt/printf.h>
#include <fmt/color.h>
namespace uipc::test
{
template <spdlog::level::level_enum Level>
RequireLog<Level>::RequireLog(::std::function<void()> check, Type type)
{
    auto sinks       = CaptureSink::instance();
    auto test_logger = CaptureSink::test_logger();

    m_last_logger = spdlog::default_logger();

    spdlog::set_default_logger(test_logger);
    check();
    spdlog::set_default_logger(m_last_logger);

    if(type == Type::Once)
    {
        success = sinks->m_msg.size() == 1 && sinks->m_msg.front().level == Level;
    }
    else if(type == Type::All)
    {
        success = std::all_of(sinks->m_msg.begin(),
                              sinks->m_msg.end(),
                              [](auto& msg) { return msg.level == Level; });
    }
    else if(type == Type::Any)
    {
        success = std::any_of(sinks->m_msg.begin(),
                              sinks->m_msg.end(),
                              [](auto& msg) { return msg.level == Level; });
    }

    auto print_captured = [&]
    {
        for(auto& msg : sinks->m_msg)
        {
            fmt::print("|> {}", msg.payload);
        }
        fmt::print("-------------------------------------------------------------------------------\n");
    };

    if(!success)
    {
        fmt::print("-------------------------------------------------------------------------------\n");
        fmt::print(fg(fmt::terminal_color::red), "RequireLog Fails. Captured messages:\n");
        print_captured();
    }
    else
    {
        fmt::print("-------------------------------------------------------------------------------\n");
        fmt::print(fg(fmt::terminal_color::green), "RequireLog Passes. Captured messages:\n");
        print_captured();
    }

    sinks->m_msg.clear();
};

}  // namespace uipc::test
