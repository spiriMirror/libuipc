#pragma once
#include <uipc/core/scene.h>
#include <uipc/core/scene_snapshot.h>
#include <uipc/geometry/attribute_collection_factory.h>

namespace uipc::core
{
class UIPC_CORE_API SceneFactory
{
    class Impl;


  public:
    SceneFactory();
    ~SceneFactory();

    [[nodiscard]] Scene         from_snapshot(const SceneSnapshot& snapshot);
    [[nodiscard]] SceneSnapshot from_json(const Json& j);
    [[nodiscard]] Json          to_json(const SceneSnapshot& scene);
    [[nodiscard]] SceneSnapshotCommit commit_from_json(const Json& json);
    [[nodiscard]] Json commit_to_json(const SceneSnapshotCommit& scene);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::core
