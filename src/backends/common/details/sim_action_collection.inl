#include "sim_action_collection.h"
namespace uipc::backend
{
template <typename F>
void SimActionCollection<F>::register_action(SimActionT&& action)
{
    UIPC_ASSERT(action.is_building(),
                "`register_sim_system()` can only be called when the SimEngine builds system.");
    if(action.is_valid())
        m_action_buffer.push_back(std::move(action));
}

template <typename F>
template <typename Callable>
void SimActionCollection<F>::register_action(ISimSystem& system, Callable&& f)
{
    m_action_buffer.emplace_back(system, std::forward<Callable>(f));
}

template <typename F>
void SimActionCollection<F>::lazy_init() const
{
    if(!built)
    {
        std::erase_if(m_action_buffer,
                      [](SimActionT& action) { return !action.is_valid(); });

        m_actions.reserve(m_action_buffer.size());
        std::ranges::move(m_action_buffer, std::back_inserter(m_actions));
        m_action_buffer.clear();
        built = true;
    }
}

template <typename F>
auto SimActionCollection<F>::view() noexcept -> span<SimActionT>
{
    lazy_init();
    return m_actions;
}

template <typename F>
auto SimActionCollection<F>::view() const noexcept -> span<const SimActionT>
{
    lazy_init();
    return m_actions;
}
}  // namespace uipc::backend
