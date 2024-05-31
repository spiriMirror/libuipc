#pragma once
#include <app/require_log.h>

namespace uipc::test
{
class Unused
{
};
constexpr Unused unused;

template <typename T>
void operator+(Unused, T&&) noexcept
{
}
}  // namespace uipc::test

#define UNUSED ::uipc::test::unused +
