#pragma once
#include <cstdint>

inline constexpr std::uint32_t UIPC_BACKEND_ABI_VERSION = 1;

struct UIPCBackendModuleInfo
{
    std::uint32_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t uipc_version_major;
    std::uint32_t uipc_version_minor;
    std::uint32_t uipc_version_patch;
    const char*   backend_name;
};
