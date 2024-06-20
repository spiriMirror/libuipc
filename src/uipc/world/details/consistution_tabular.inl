namespace uipc::world
{
template <std::derived_from<IConstitution> T, typename... Args>
T& ConstitutionTabular::create(Args&&... args)
{
    auto& p =
        m_constitutions.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
    m_types.insert(p->type());
    return static_cast<T&>(*p);
}
}  // namespace uipc::world
