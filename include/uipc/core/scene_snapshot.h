#pragma once
#include <uipc/core/scene.h>
#include <uipc/geometry/geometry_commit.h>
#include <uipc/core/object_collection.h>

namespace uipc::core
{
/**
 * Create a scene snapshot from the given scene, which is a plain data
 * copy of the scene (detached from the world).
 */
class UIPC_CORE_API SceneSnapshot
{
    friend class Scene;
    friend class SceneSnapshotCommit;
    friend class SceneFactory;

  public:
    SceneSnapshot(const Scene& scene);
    SceneSnapshot(const SceneSnapshot&)            = default;
    SceneSnapshot(SceneSnapshot&&)                 = default;
    SceneSnapshot& operator=(const SceneSnapshot&) = default;
    SceneSnapshot& operator=(SceneSnapshot&&)      = default;

  private:
    SceneSnapshot() = default;
    Json                     m_config;
    ObjectCollectionSnapshot m_object_collection;
    vector<ContactElement>   m_contact_elements;


    unordered_map<IndexT, S<geometry::Geometry>> m_geometries;
    unordered_map<IndexT, S<geometry::Geometry>> m_rest_geometries;

    geometry::AttributeCollection m_contact_models;
};

/**
 * SceneSnapCommit (from B to A) = SceneSnapshotA - SceneSnapshotB 
 */
class UIPC_CORE_API SceneSnapshotCommit
{
    friend SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst,
                                                       const SceneSnapshot& src);
    friend class internal::Scene;

  private:
    SceneSnapshotCommit(const SceneSnapshot& dst, const SceneSnapshot& src);

  private:
    // Fully Copy:
    Json                     m_config;
    ObjectCollectionSnapshot m_object_collection;
    vector<ContactElement>   m_contact_elements;

    // Full Copy Geometries/ Diff Copy AttributeCollection
    unordered_map<IndexT, geometry::GeometryCommit> m_diff_geometries;
    unordered_map<IndexT, geometry::GeometryCommit> m_rest_diff_geometries;

    // Diff Copy AttributeCollection
    geometry::AttributeCollectionCommit m_contact_models;
};

SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst,
                                            const SceneSnapshot& src);
}  // namespace uipc::core
