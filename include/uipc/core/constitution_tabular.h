#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/common/span.h>
#include <uipc/common/set.h>

namespace uipc::core
{
class UIPC_CORE_API ConstitutionTabular
{
  public:
    ConstitutionTabular() noexcept;
    ~ConstitutionTabular() noexcept;
    // delete copy
    ConstitutionTabular(const ConstitutionTabular&)            = delete;
    ConstitutionTabular& operator=(const ConstitutionTabular&) = delete;

    void      insert(const constitution::IConstitution& constitution);
    span<U64> uids() const noexcept;
    const set<std::string>& types() const noexcept;

  private:
    class Impl;
    U<Impl> m_impl;
    friend class World;
    void insert(U64 uid);
};
}  // namespace uipc::core