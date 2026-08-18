#pragma once
#include <cstdlib>
#include <cassert>
#include <muda/tools/fuzzy.h>
#include <muda/assert.h>
#include <muda/print.h>
#include <muda/muda_config.h>
#include <muda/muda_def.h>

#ifdef __CUDA_ARCH__
#define MUDA_KERNEL_PRINT(fmt, ...)                                            \
    {                                                                          \
        if(muda::block_dim().y == 1 && muda::block_dim().z == 1)               \
        {                                                                      \
            ::muda::print("(%d|%d)-(%d|%d):" fmt "\n",                         \
                          muda::block_idx().x,                                 \
                          muda::grid_dim().x,                                  \
                          muda::thread_idx().x,                                \
                          muda::block_dim().x,                                 \
                          ##__VA_ARGS__);                                      \
        }                                                                      \
        else if(muda::block_dim().z == 1)                                      \
        {                                                                      \
            ::muda::print("(%d|%d,%d|%d)-(%d|%d,%d|%d):" fmt "\n",             \
                          muda::block_idx().x,                                 \
                          muda::grid_dim().x,                                  \
                          muda::block_idx().y,                                 \
                          muda::grid_dim().y,                                  \
                          muda::thread_idx().x,                                \
                          muda::block_dim().x,                                 \
                          muda::thread_idx().y,                                \
                          muda::block_dim().y,                                 \
                          ##__VA_ARGS__);                                      \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            ::muda::print("(%d|%d,%d|%d,%d|%d)-(%d|%d,%d|%d,%d|%d):" fmt "\n", \
                          muda::block_idx().x,                                 \
                          muda::grid_dim().x,                                  \
                          muda::block_idx().y,                                 \
                          muda::grid_dim().y,                                  \
                          muda::block_idx().z,                                 \
                          muda::grid_dim().z,                                  \
                          muda::thread_idx().x,                                \
                          muda::block_dim().x,                                 \
                          muda::thread_idx().y,                                \
                          muda::block_dim().y,                                 \
                          muda::thread_idx().z,                                \
                          muda::block_dim().z,                                 \
                          ##__VA_ARGS__);                                      \
        }                                                                      \
    }
#else
#define MUDA_KERNEL_PRINT(fmt, ...)                                            \
    {                                                                          \
        ::muda::print("(host):" fmt "\n", ##__VA_ARGS__);                      \
    }
#endif

//when muda::TRAP_ON_ERROR == true trap the device
#define MUDA_DEBUG_TRAP()                                                      \
    {                                                                          \
        if constexpr(::muda::TRAP_ON_ERROR)                                    \
            ::muda::trap();                                                    \
    }

// check whether (res == true), if not, print the error info (when muda::TRAP_ON_ERROR == true
// trap the device)
#define MUDA_KERNEL_ASSERT(res, fmt, ...)                                         \
    {                                                                             \
        if constexpr(::muda::RUNTIME_CHECK_ON)                                    \
        {                                                                         \
            if(!(res))                                                            \
            {                                                                     \
                MUDA_KERNEL_PRINT("%s(%d): %s:\n <assert> " #res " failed. " fmt, \
                                  __FILE__,                                       \
                                  __LINE__,                                       \
                                  MUDA_FUNCTION_SIG,                              \
                                  ##__VA_ARGS__);                                 \
                MUDA_DEBUG_TRAP();                                                \
            }                                                                     \
        }                                                                         \
    }

// check whether (res == true), if not, print the error info(never trap the device)
#define MUDA_KERNEL_CHECK(res, fmt, ...)                                         \
    {                                                                            \
        if constexpr(::muda::RUNTIME_CHECK_ON)                                   \
        {                                                                        \
            if(!(res))                                                           \
            {                                                                    \
                MUDA_KERNEL_PRINT("%s(%d): %s:\n <check> " #res " failed. " fmt, \
                                  __FILE__,                                      \
                                  __LINE__,                                      \
                                  MUDA_FUNCTION_SIG,                             \
                                  ##__VA_ARGS__);                                \
            }                                                                    \
        }                                                                        \
    }

// print error info, and call muda_debug_trap()
#define MUDA_KERNEL_ERROR(fmt, ...)                                            \
    {                                                                          \
        MUDA_KERNEL_PRINT("<error> " fmt, ##__VA_ARGS__);                      \
        MUDA_DEBUG_TRAP();                                                     \
    }

#define MUDA_KERNEL_ERROR_WITH_LOCATION(fmt, ...)                                                              \
    {                                                                                                          \
        MUDA_KERNEL_PRINT("%s(%d): %s:\n <error> " fmt, __FILE__, __LINE__, MUDA_FUNCTION_SIG, ##__VA_ARGS__); \
        MUDA_DEBUG_TRAP();                                                                                     \
    }

// print warn info
#define MUDA_KERNEL_WARN(fmt, ...)                                             \
    {                                                                          \
        MUDA_KERNEL_PRINT("<warn>" fmt, ##__VA_ARGS__);                        \
    }

#define MUDA_KERNEL_WARN_WITH_LOCATION(fmt, ...)                                                             \
    {                                                                                                        \
        MUDA_KERNEL_PRINT("%s(%d): %s:\n <warn>" fmt, __FILE__, __LINE__, MUDA_FUNCTION_SIG, ##__VA_ARGS__); \
    }


#define MUDA_ASSERT(res, fmt, ...) MUDA_KERNEL_ASSERT(res, fmt, ##__VA_ARGS__)

#define MUDA_ERROR(fmt, ...) MUDA_KERNEL_ERROR(fmt, ##__VA_ARGS__)

#define MUDA_ERROR_WITH_LOCATION(fmt, ...) MUDA_KERNEL_ERROR_WITH_LOCATION(fmt, ##__VA_ARGS__)