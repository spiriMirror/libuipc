#include <sanity_checker_collection.h>
#include <sanity_checker_auto_register.h>

namespace uipc::sanity_check
{
SanityCheckerCollection::SanityCheckerCollection(std::string_view workspace) noexcept
{
    m_workspace = fmt::format("{}/sanity_check/", workspace);
}

SanityCheckerCollection::~SanityCheckerCollection() {}

std::string_view SanityCheckerCollection::workspace() const noexcept
{
    return m_workspace;
}

void SanityCheckerCollection::init(core::Scene& s)
{
    for(const auto& creator : SanityCheckerAutoRegister::creators().entries)
    {
        m_entries.emplace_back(creator(*this, s));
    }

    for(const auto& entry : m_entries)
    {
        entry->init();
    }
}

SanityCheckResult SanityCheckerCollection::check() const
{
    int result = static_cast<int>(SanityCheckResult::Success);
    for(const auto& entry : m_entries)
    {
        int check = static_cast<int>(entry->check());

        if(check > result)
            result = check;
    }

    for(const auto& entry : m_entries)
    {
        entry->deinit();
    }

    return static_cast<SanityCheckResult>(result);
}
}  // namespace uipc::sanity_check
