#pragma once
#include <app/require_log.h>
#include <uipc/common/json.h>
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

class Scene
{
  public:
    static Json default_config();
};
}  // namespace uipc::test

#define UNUSED ::uipc::test::unused +
