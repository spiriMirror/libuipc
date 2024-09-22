#pragma once
#include <functional>
#include <backends/common/i_sim_system.h>

namespace uipc::backend
{
class SimSystem;

template <typename F = void()>
class SimAction
{
};

template <typename R, typename... Args>
class SimAction<R(Args...)>
{
  public:
    using Callable = std::function<R(Args...)>;

    SimAction(ISimSystem& sim_system, Callable&& A) noexcept;

    R operator()(Args... args) const;

    ISimSystem&       owner() noexcept;
    const ISimSystem& owner() const noexcept;
    bool              is_valid() const noexcept;
    bool              is_building() const noexcept;

  private:
    ISimSystem& m_sim_system;
    Callable    m_action;
};
}  // namespace uipc::backend

#include "details/sim_action.inl"