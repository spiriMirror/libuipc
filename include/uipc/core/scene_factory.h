#pragma once
#include <uipc/core/scene.h>

namespace uipc::core
{
class UIPC_CORE_API SceneFactory
{
    class Impl;


  public:
    SceneFactory(const Json& config = default_config());
    ~SceneFactory();

    [[nodiscard]] S<Scene> from_json(const Json& j);
    [[nodiscard]] Json     to_json(const Scene& scene);

    class SceneCommit;

    static Json default_config();

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::core
