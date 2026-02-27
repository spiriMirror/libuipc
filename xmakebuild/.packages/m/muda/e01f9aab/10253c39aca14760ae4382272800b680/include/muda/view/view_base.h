#pragma once
#include <muda/type_traits/type_modifier.h>

namespace muda
{
template <bool IsConst_>
class ViewBase
{
  public:
    constexpr static bool IsConst    = IsConst_;
    constexpr static bool IsNonConst = !IsConst_;
    template <typename T>
    using auto_const_t = std::conditional_t<IsConst, const T, T>;
    template <typename T>
    using non_const_enable_t = std::enable_if_t<IsNonConst, T>;

  private:
    std::byte _dummy; // a workaround to avoid NVCC EBO bug.
};
}  // namespace muda