#include <sim_action.h>

namespace uipc::backend::cuda
{
SimAction::SimAction(SimSystem& sim_system, std::function<void()>&& action) noexcept
    : m_sim_system(sim_system)
    , m_action(std::move(action))
{
}

SimSystem& SimAction::owner() noexcept
{
    return m_sim_system;
}

const SimSystem& SimAction::owner() const noexcept
{
    return m_sim_system;
}
}  // namespace uipc::backend::cuda
