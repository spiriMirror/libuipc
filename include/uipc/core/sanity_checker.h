#pragma once
#include <uipc/core/i_sanity_checker.h>

namespace uipc::core
{
class Scene;

class UIPC_CORE_API SanityChecker final
{
  public:
    SanityChecker(Scene& scene);
    ~SanityChecker();

    SanityCheckResult check(std::string_view workspace);
    void              report();

    const unordered_map<U64, S<SanityCheckMessage>>& errors() const;
    const unordered_map<U64, S<SanityCheckMessage>>& warns() const;
    const unordered_map<U64, S<SanityCheckMessage>>& infos() const;

    void clear();

  private:
    core::SanityCheckMessageCollection m_errors;
    core::SanityCheckMessageCollection m_warns;
    core::SanityCheckMessageCollection m_infos;

    Scene& m_scene;
};
}  // namespace uipc::core
