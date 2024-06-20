#include <uipc/world/constitution_tabular.h>
#include <algorithm>

namespace uipc::world
{
span<U64> ConstitutionTabular::uids() const noexcept
{
    sort_if_needed();
    return m_uids;
}

const set<ConstitutionTypes>& ConstitutionTabular::types() const noexcept
{
    return m_types;
}

void ConstitutionTabular::sort_if_needed() const noexcept
{
    if(m_is_sorted)
        return;

    m_uids.resize(m_constitutions.size());
    std::transform(m_constitutions.begin(),
                   m_constitutions.end(),
                   m_uids.begin(),
                   [](const U<IConstitution>& constitution)
                   { return constitution->uid(); });

    std::sort(m_uids.begin(), m_uids.end());

    m_is_sorted = true;
}
}  // namespace uipc::world
