#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/common/span.h>
#include <uipc/common/set.h>

namespace uipc::backend
{
class SceneVisitor;
}

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::core
{
class UIPC_CORE_API ConstitutionTabular
{
    friend class internal::Scene;

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
    friend class Scene;
    void init(internal::Scene& scene);  // only be called by Scene.
};
}  // namespace uipc::core