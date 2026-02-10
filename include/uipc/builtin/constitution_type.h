#pragma once
#include <string_view>

namespace uipc::builtin
{
constexpr std::string_view AffineBody    = "AffineBody";
constexpr std::string_view FiniteElement = "FiniteElement";

constexpr std::string_view Constraint = "Constraint";

constexpr std::string_view InterAffineBody = "InterAffineBody";
constexpr std::string_view InterPrimitive  = "InterPrimitive";
}  // namespace uipc::builtin
