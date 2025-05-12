#pragma once
#include <uipc/core/scene.h>

namespace uipc::core
{
class UIPC_CORE_API SceneFactory
{
    class Impl;

  public:
    SceneFactory();
    ~SceneFactory();

    [[nodiscard]] Scene from_json(const Json& j);
    [[nodiscard]] Json  to_json(const Scene& scene);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::core
