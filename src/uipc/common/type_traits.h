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


template <typename Sig>
struct signature;

template <typename R, typename... Args>
struct signature<R(Args...)>
{
    using type = std::tuple<R, Args...>;
};

template <typename R, typename... Args>
struct signature<R (*)(Args...)> : signature<R(Args...)>
{
};

template <typename R, typename... Args>
struct signature<R (&)(Args...)> : signature<R(Args...)>
{
};

template <typename R, typename C, typename... Args>
struct signature<R (C::*)(Args...)> : signature<R(Args...)>
{
};

template <typename R, typename C, typename... Args>
struct signature<R (C::*)(Args...) const> : signature<R(Args...)>
{
};

template <typename R, typename C, typename... Args>
struct signature<R (C::*)(Args...) volatile> : signature<R(Args...)>
{
};

template <typename R, typename C, typename... Args>
struct signature<R (C::*)(Args...) const volatile> : signature<R(Args...)>
{
};

template <typename R, typename C, typename... Args>
struct signature<R (C::*)(Args...) &&> : signature<R(Args...)>
{
};

template <typename Sig>
using signature_t = typename signature<Sig>::type;
}  // namespace uipc
