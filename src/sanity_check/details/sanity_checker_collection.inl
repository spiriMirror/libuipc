namespace uipc::sanity_check
{
template <std::derived_from<core::ISanityChecker> SanityCheckerT>
inline SanityCheckerT* SanityCheckerCollection::find() const
{
    for(const auto& entry : m_entries)
    {
        if(auto* p = dynamic_cast<SanityCheckerT*>(entry.get()))
        {
            return p;
        }
    }
    return nullptr;
}
}  // namespace uipc::sanity_check