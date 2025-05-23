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

    S<geometry::AttributeCollection> m_contact_models;
};

/**
 * SceneSnapCommit (from B to A) = SceneSnapshotA - SceneSnapshotB 
 */
class UIPC_CORE_API SceneSnapshotCommit
{
    friend class SceneFactory;
    friend SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst,
                                                       const SceneSnapshot& src);
    friend class internal::Scene;

  public:
    SceneSnapshotCommit() = default;
    SceneSnapshotCommit(const SceneSnapshot& dst, const SceneSnapshot& src);

    bool        is_valid() const noexcept { return m_is_valid; }
    const Json& config() const noexcept { return m_config; }
    const ObjectCollectionSnapshot& object_collection() const noexcept
    {
        return m_object_collection;
    }

    const vector<ContactElement>& contact_elements() const noexcept
    {
        return m_contact_elements;
    }

    const unordered_map<IndexT, S<geometry::GeometryCommit>>& geometries() const noexcept
    {
        return m_geometries;
    }

    const unordered_map<IndexT, S<geometry::GeometryCommit>>& rest_geometries() const noexcept
    {
        return m_rest_geometries;
    }

    const geometry::AttributeCollectionCommit& contact_models() const noexcept
    {
        return *m_contact_models;
    }

  private:
    bool m_is_valid = true;
    // Fully Copy:
    Json                     m_config;
    ObjectCollectionSnapshot m_object_collection;
    vector<ContactElement>   m_contact_elements;

    // Full Copy Geometries/ Diff Copy AttributeCollection
    unordered_map<IndexT, S<geometry::GeometryCommit>> m_geometries;
    unordered_map<IndexT, S<geometry::GeometryCommit>> m_rest_geometries;

    // Diff Copy AttributeCollection
    S<geometry::AttributeCollectionCommit> m_contact_models;
};

SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst,
                                            const SceneSnapshot& src);
}  // namespace uipc::core
