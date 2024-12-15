#pragma once
#include <string_view>

#define UIPC_BUILTIN_ATTRIBUTE(name) constexpr std::string_view name = #name

namespace uipc::builtin
{
#include <uipc/builtin/details/attribute_name.h>
}  // namespace uipc::builtin

#undef UIPC_BUILTIN_ATTRIBUTE
