#pragma once
#include <uipc/core/scene.h>
#include <uipc/geometry/attribute_collection_factory.h>

namespace uipc::core
{
/**
 * Create a scene snapshot from the given scene, which is a plain data
 * copy of the scene (detached from the world).
 */
class UIPC_CORE_API SceneSnapshot
{
    friend class Scene;
    friend class SceneSnapCommit;
    friend class SceneFactory;

  public:
    class Object
    {
        friend class SceneSnapshot;
        friend class SceneFactory;
        IndexT         m_id;
        std::string    m_name;
        vector<IndexT> m_geometries;
    };

    SceneSnapshot(const Scene& scene);
    SceneSnapshot(const SceneSnapshot&)            = default;
    SceneSnapshot(SceneSnapshot&&)                 = default;
    SceneSnapshot& operator=(const SceneSnapshot&) = default;
    SceneSnapshot& operator=(SceneSnapshot&&)      = default;

  private:
    SceneSnapshot() = default;
    Json                          m_config;
    vector<SceneSnapshot::Object> m_objects;
    vector<ContactElement>        m_contact_elements;


    unordered_map<IndexT, S<geometry::Geometry>> m_geometries;
    unordered_map<IndexT, S<geometry::Geometry>> m_rest_geometries;

    geometry::AttributeCollection m_contact_models;
};

/**
 * SceneSnapCommit (from B to A) = SceneSnapshotA - SceneSnapshotB 
 */
class UIPC_CORE_API SceneSnapCommit
{
    friend SceneSnapCommit UIPC_CORE_API operator-(const SceneSnapshot& dst,
                                                   const SceneSnapshot& src);

  private:
    SceneSnapCommit(const SceneSnapshot& dst, const SceneSnapshot& src);

  private:
    // Fully Copy:
    Json                          m_config;
    vector<SceneSnapshot::Object> m_objects;
    vector<ContactElement>        m_contact_elements;

    // Full Copy Geometries/ Diff Copy AttributeCollection
    unordered_map<IndexT, geometry::GeometryCommit> m_diff_geometries;
    unordered_map<IndexT, geometry::GeometryCommit> m_rest_diff_geometries;

    // Diff Copy AttributeCollection
    geometry::AttributeCollectionCommit m_contact_models;
};

SceneSnapCommit UIPC_CORE_API operator-(const SceneSnapshot& dst, const SceneSnapshot& src);

class UIPC_CORE_API SceneFactory
{
    class Impl;


  public:
    SceneFactory();
    ~SceneFactory();

    [[nodiscard]] Scene         from_snapshot(const SceneSnapshot& snapshot);
    [[nodiscard]] SceneSnapshot from_json(const Json& j);
    [[nodiscard]] Json          to_json(const SceneSnapshot& scene);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::core
