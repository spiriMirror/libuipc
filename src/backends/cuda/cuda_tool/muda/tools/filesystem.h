#pragma once
#include <muda/tools/platform.h>

#ifdef MUDA_PLATFORM_LINUX
#if __GNUC__ >= 8
#include <filesystem>
namespace muda
{
    namespace filesystem = std::filesystem;
}
#else
#include <experimental/filesystem>
namespace muda
{
    namespace filesystem = std::experimental::filesystem;
}
#endif
#endif

#ifdef MUDA_PLATFORM_WINDOWS
#include <filesystem>
namespace muda
{
    namespace filesystem = std::filesystem;
}
#endif
