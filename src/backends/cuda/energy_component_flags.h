#pragma once
#include <type_define.h>
#include <uipc/common/flag.h>

namespace uipc::backend::cuda
{
enum class EnergyComponentFlags : uint32_t
{
    None = 0,
    // Contact Part
    Contact = 1,
    // NonContact Part
    Complement = 1 << 1,
    All        = Contact | Complement
};
}

// Add specialization to customize namespace to mark the enum as a flag type
// This enables functionality like combined name generation like "Contact|Complement"
template <>
struct magic_enum::customize::enum_range<uipc::backend::cuda::EnergyComponentFlags>
{
    static constexpr bool is_flags = true;
};