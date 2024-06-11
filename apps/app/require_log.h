#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <uipc/common/list.h>
#include <mutex>

namespace uipc::test
{
class CaptureSink : public spdlog::sinks::base_sink<std::mutex>
{
    struct LogMsg
    {
        spdlog::level::level_enum level;
        std::string               payload;
    };

  public:
    CaptureSink() = default;

    void sink_it_(const spdlog::details::log_msg& msg) override;
    void flush_() override {}

  private:
    template <spdlog::level::level_enum Level>
    friend class RequireLog;
    static std::shared_ptr<CaptureSink> instance();
    // a test logger that uses the CaptureSink
    static std::shared_ptr<spdlog::logger> test_logger();
    list<LogMsg>                      m_msg;
};


template <spdlog::level::level_enum Level>
class RequireLog
{
  public:
    enum class Type
    {
        Once,
        All,
        Any
    };

  public:
    RequireLog(::std::function<void()> check, Type type);

    bool success = false;

  private:
    std::shared_ptr<spdlog::logger> m_last_logger;
};

}  // namespace uipc::test

#define REQUIRE_ONCE_INFO(...)                                                 \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::info>(                   \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::info>::Type::Once)   \
                .success)

#define REQUIRE_ALL_INFO(...)                                                  \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::info>(                   \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::info>::Type::All)    \
                .success)

#define REQUIRE_HAS_INFO(...)                                                  \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::info>(                   \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::info>::Type::Any)    \
                .success)


#define REQUIRE_ONCE_WARN(...)                                                 \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::warn>(                   \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::warn>::Type::Once)   \
                .success)

#define REQUIRE_ALL_WARN(...)                                                  \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::warn>(                   \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::warn>::Type::All)    \
                .success)

#define REQUIRE_HAS_WARN(...)                                                  \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::warn>(                   \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::warn>::Type::Any)    \
                .success)

#define REQUIRE_ONCE_ERROR(...)                                                \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::err>(                    \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::err>::Type::Once)    \
                .success)

#define REQUIRE_ALL_ERROR(...)                                                 \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::err>(                    \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::err>::Type::All)     \
                .success)

#define REQUIRE_HAS_ERROR(...)                                                 \
    REQUIRE(::uipc::test::RequireLog<::spdlog::level::err>(                    \
                [&]() { __VA_ARGS__; },                                        \
                ::uipc::test::RequireLog<::spdlog::level::err>::Type::Any)     \
                .success)

#include "details/require_log.inl"