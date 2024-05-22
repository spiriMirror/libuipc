#pragma once
#include <spdlog/spdlog.h>

#define UIPC_INFO_WITH_LOCATION(fmt, ...)                                      \
    {                                                                          \
        spdlog::info("[{}({})] {}" fmt, __FILE__, __LINE__, __VA_ARGS__);      \
    }

#define UIPC_WARN_WITH_LOCATION(fmt, ...)                                      \
    {                                                                          \
        spdlog::warn("[{}({})] {}" fmt, __FILE__, __LINE__, __VA_ARGS__);      \
    }

#define UIPC_ERROR_WITH_LOCATION(fmt, ...)                                     \
    {                                                                          \
        spdlog::error("[{}({})] {}" fmt, __FILE__, __LINE__, __VA_ARGS__);     \
    }

#define UIPC_ASSERT(condition, fmt, ...)                                       \
    {                                                                          \
        if(!(condition))                                                       \
        {                                                                      \
            spdlog::error("[{}({})] Assertion: " #condition ", failed.\n" fmt, \
                          __FILE__,                                            \
                          __LINE__,                                            \
                          __VA_ARGS__);                                        \
            std::abort();                                                      \
        }                                                                      \
    }
