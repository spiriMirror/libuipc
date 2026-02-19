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

/**
 * @brief Get the string representation of EnergyComponentFlags
 * 
 * A wrapper around magic_enum::enum_flags_name, on NVCC we can't use:
 * magic_enum::enum_flags_name(flags) directly due to compilation issues.
 */
std::string enum_flags_name(EnergyComponentFlags flags);
}  // namespace uipc::backend::cuda

namespace magic_enum::customize
{
template <>
struct enum_range<uipc::backend::cuda::EnergyComponentFlags>
{
    static constexpr bool is_flags = true;
};
}  // namespace magic_enum::customize