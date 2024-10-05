#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>
#include <uipc/common/set.h>
#include <uipc/common/list.h>

namespace uipc::core
{
class UIPC_CORE_API ConstitutionTabular
{
  public:
    ConstitutionTabular() = default;

    // delete copy
    ConstitutionTabular(const ConstitutionTabular&)            = delete;
    ConstitutionTabular& operator=(const ConstitutionTabular&) = delete;

    void insert(const constitution::IConstitution& constitution);

    span<U64> uids() const noexcept;

    const set<std::string>& types() const noexcept;

  private:
    friend class World;
    void                insert(U64 uid);
    mutable bool        m_is_sorted = false;
    mutable vector<U64> m_uids;
    set<U64>            m_uid_set;
    void                sort_if_needed() const noexcept;
    set<std::string>    m_types;
};
}  // namespace uipc::core