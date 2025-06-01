#pragma once
#include <spdlog/spdlog.h>
#include <uipc/common/string.h>
#include <uipc/common/config.h>
#include <uipc/common/abort.h>

#define UIPC_LOG_WITH_LOCATION(level, ...)                                     \
    {                                                                          \
        ::uipc::string msg = ::fmt::format(__VA_ARGS__);                       \
        spdlog::log((level), "{} {}({})", msg, __FILE__, __LINE__);            \
    }

#define UIPC_INFO_WITH_LOCATION(...)                                           \
    UIPC_LOG_WITH_LOCATION(spdlog::level::info, __VA_ARGS__)

#define UIPC_WARN_WITH_LOCATION(...)                                           \
    UIPC_LOG_WITH_LOCATION(spdlog::level::warn, __VA_ARGS__)

#define UIPC_ERROR_WITH_LOCATION(...)                                          \
    UIPC_LOG_WITH_LOCATION(spdlog::level::err, __VA_ARGS__)

#define UIPC_ASSERT(condition, ...)                                                       \
    {                                                                                     \
        if(!(condition))                                                                  \
        {                                                                                 \
            ::uipc::string msg = ::fmt::format(__VA_ARGS__);                              \
            ::uipc::string assert_meg =                                                   \
                ::fmt::format("Assertion " #condition " failed. {}", msg);                \
            spdlog::log(spdlog::level::err, "{} {}({})", assert_meg, __FILE__, __LINE__); \
            ::uipc::abort();                                                              \
        }                                                                                 \
    }
