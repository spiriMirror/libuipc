namespace uipc::world
{
template <std::derived_from<IConstitution> T, typename... Args>
T& ConstitutionTabular::create(Args&&... args)
{
    m_constitutions.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
    return *static_cast<T*>(m_constitutions.back().get());
}
}  // namespace uipc::world
