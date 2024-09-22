#include <uipc/world/constitution_tabular.h>
#include <uipc/builtin/constitution_uid_collection.h>
#include <algorithm>

namespace uipc::world
{
void ConstitutionTabular::insert(const constitution::IConstitution& constitution)
{
    UIPC_ASSERT(!m_is_sorted, "Cannot insert into a built ConstitutionTabular");
    m_uid_set.insert(constitution.uid());
    m_types.insert(std::string{constitution.type()});
}

span<U64> ConstitutionTabular::uids() const noexcept
{
    sort_if_needed();
    return m_uids;
}

const set<std::string>& ConstitutionTabular::types() const noexcept
{
    return m_types;
}

void ConstitutionTabular::insert(U64 uid)
{
    const auto& uid_info = builtin::ConstitutionUIDCollection::instance().find(uid);
    m_uid_set.insert(uid);
    m_types.insert(std::string{uid_info.type});
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
