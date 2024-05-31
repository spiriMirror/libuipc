#pragma once
#include <type_traits>

namespace uipc
{
template <typename T, typename DstT>
struct propagate_const
{
  private:
    using DstT_ = std::remove_cv_t<DstT>;

  public:
    using type = std::conditional_t<std::is_const_v<T>, const DstT_, DstT_>;
};

template <typename T, typename DstT>
using propagate_const_t = typename propagate_const<T, DstT>::type;
}  // namespace uipc
