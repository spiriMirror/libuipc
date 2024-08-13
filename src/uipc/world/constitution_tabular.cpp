#include <uipc/world/constitution_tabular.h>
#include <algorithm>

namespace uipc::world
{
void ConstitutionTabular::insert(const constitution::IConstitution& constitution)
{
    UIPC_ASSERT(!m_is_sorted, "Cannot insert into a built ConstitutionTabular");
    m_uid_set.insert(constitution.uid());
    m_types.insert(constitution.type());
}

span<U64> ConstitutionTabular::uids() const noexcept
{
    sort_if_needed();
    return m_uids;
}

const set<constitution::ConstitutionType>& ConstitutionTabular::types() const noexcept
{
    return m_types;
}

void ConstitutionTabular::sort_if_needed() const noexcept
{
    if(m_is_sorted)
        return;

    m_uids.resize(m_uid_set.size());
    std::ranges::copy(m_uid_set, m_uids.begin());
    std::sort(m_uids.begin(), m_uids.end());

    m_is_sorted = true;
}
}  // namespace uipc::world
