#pragma once
#include <uipc/core/i_sanity_checker.h>

namespace uipc::core::internal
{
class Scene;
class Engine;
}

namespace uipc::core
{
class UIPC_CORE_API SanityChecker final
{
  public:
    SanityChecker(internal::Scene& scene, internal::Engine& engine);
    ~SanityChecker();

    SanityCheckResult check();
    void              report();

    const unordered_map<U64, S<SanityCheckMessage>>& errors() const;
    const unordered_map<U64, S<SanityCheckMessage>>& warns() const;
    const unordered_map<U64, S<SanityCheckMessage>>& infos() const;

    void clear();

  private:
    core::SanityCheckMessageCollection m_errors;
    core::SanityCheckMessageCollection m_warns;
    core::SanityCheckMessageCollection m_infos;

    internal::Scene&  m_scene;
    internal::Engine& m_engine;
};
}  // namespace uipc::core
