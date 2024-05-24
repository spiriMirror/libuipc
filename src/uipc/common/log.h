#pragma once
#include <spdlog/spdlog.h>

#define UIPC_INFO_WITH_LOCATION(fmt, ...)                                      \
    {                                                                          \
        spdlog::info(fmt " [{}({})]", __VA_ARGS__, __FILE__, __LINE__);        \
    }

#define UIPC_WARN_WITH_LOCATION(fmt, ...)                                      \
    {                                                                          \
        spdlog::warn(fmt " [{}({})]", __VA_ARGS__, __FILE__, __LINE__);        \
    }

#define UIPC_ERROR_WITH_LOCATION(fmt, ...)                                     \
    {                                                                          \
        spdlog::error(fmt " [{}({})]", __VA_ARGS__, __FILE__, __LINE__);       \
    }

#define UIPC_ASSERT(condition, fmt, ...)                                        \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            spdlog::error("Assertion: " #condition ", failed. " fmt "[{}({})]", \
                          __VA_ARGS__,                                          \
                          __FILE__,                                             \
                          __LINE__);                                            \
            std::abort();                                                       \
        }                                                                       \
    }
