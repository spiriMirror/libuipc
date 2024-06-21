#pragma once
#include <spdlog/spdlog.h>
#include <uipc/common/config.h>
#define UIPC_INFO_WITH_LOCATION(...)                                           \
    {                                                                          \
        SPDLOG_INFO(__VA_ARGS__);                                              \
    }

#define UIPC_WARN_WITH_LOCATION(...)                                           \
    {                                                                          \
        SPDLOG_WARN(__VA_ARGS__);                                              \
    }

#define UIPC_ERROR_WITH_LOCATION(...)                                          \
    {                                                                          \
        SPDLOG_ERROR(__VA_ARGS__);                                             \
    }

#define UIPC_ASSERT(condition, ...)                                            \
    if constexpr(::uipc::RUNTIME_CHECK)                                        \
    {                                                                          \
        if(!(condition))                                                       \
        {                                                                      \
            ::std::string msg = ::fmt::format(__VA_ARGS__);                    \
            ::std::string assert_meg =                                         \
                ::fmt::format("Assertion " #condition " failed. {}", msg);     \
            SPDLOG_ERROR(assert_meg);                                          \
            ::std::abort();                                                    \
        }                                                                      \
    }
