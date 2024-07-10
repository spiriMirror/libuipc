namespace uipc::backend
{
template <typename R, typename... Args>
SimAction<R(Args...)>::SimAction(ISimSystem& sim_system, Callable&& A) noexcept
    : m_sim_system{sim_system}
    , m_action{std::move(A)}
{
}

template <typename R, typename... Args>
R SimAction<R(Args...)>::operator()(Args... args) const
{
    return m_action(std::forward<Args>(args)...);
}

template <typename R, typename... Args>
ISimSystem& SimAction<R(Args...)>::owner() noexcept
{
    return m_sim_system;
}

template <typename R, typename... Args>
inline const ISimSystem& SimAction<R(Args...)>::owner() const noexcept
{
    return m_sim_system;
}

template <typename R, typename... Args>
bool SimAction<R(Args...)>::is_valid() const noexcept
{
    return m_sim_system.is_valid();
}

template <typename R, typename... Args>
bool SimAction<R(Args...)>::is_building() const noexcept
{
    return m_sim_system.is_building();
}
}  // namespace uipc::backend
