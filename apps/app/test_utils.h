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
    static void dump_config(const Json& config, std::string_view workspace);
};


}  // namespace uipc::test

#define UNUSED ::uipc::test::unused +
