#pragma once
#include <functional>

namespace uipc::backend::cuda
{
class SimSystem;

class SimAction
{
  public:
    SimAction(SimSystem& sim_system, std::function<void()>&& action) noexcept;
    inline void      operator()() const { m_action(); }
    SimSystem&       owner() noexcept;
    const SimSystem& owner() const noexcept;

  private:
    SimSystem&            m_sim_system;
    std::function<void()> m_action;
};
}  // namespace uipc::backend::cuda
