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

    S<geometry::AttributeCollection> m_config;

    ObjectCollectionSnapshot m_object_collection;

    unordered_map<IndexT, S<geometry::Geometry>> m_geometries;
    unordered_map<IndexT, S<geometry::Geometry>> m_rest_geometries;
    IndexT                                       m_geometry_next_id      = -1;
    IndexT                                       m_rest_geometry_next_id = -1;

    vector<ContactElement>  m_contact_elements;
    vector<SubsceneElement> m_subscene_elements;

    S<geometry::AttributeCollection> m_contact_models;
    S<geometry::AttributeCollection> m_subscene_models;
    bool                             m_contact_default_model_user_set = false;
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

    bool is_valid() const noexcept { return m_is_valid; }

    const geometry::AttributeCollectionCommit& config() const noexcept
    {
        return *m_config;
    }

    const ObjectCollectionSnapshot& object_collection() const noexcept
    {
        return m_object_collection;
    }

    const vector<ContactElement>& contact_elements() const noexcept
    {
        return m_contact_elements;
    }

    const vector<SubsceneElement>& subscene_elements() const noexcept
    {
        return m_subscene_elements;
    }

    const unordered_map<IndexT, S<geometry::GeometryCommit>>& geometries() const noexcept
    {
        return m_geometries;
    }

    const unordered_map<IndexT, S<geometry::GeometryCommit>>& rest_geometries() const noexcept
    {
        return m_rest_geometries;
    }

    span<const IndexT> removed_geometry_ids() const noexcept
    {
        return m_removed_geometry_ids;
    }

    span<const IndexT> removed_rest_geometry_ids() const noexcept
    {
        return m_removed_rest_geometry_ids;
    }

    IndexT geometry_next_id() const noexcept { return m_geometry_next_id; }
    IndexT rest_geometry_next_id() const noexcept
    {
        return m_rest_geometry_next_id;
    }

    const geometry::AttributeCollectionCommit& contact_models() const noexcept
    {
        return *m_contact_models;
    }

    const geometry::AttributeCollectionCommit& subscene_models() const noexcept
    {
        return *m_subscene_models;
    }

    bool contact_default_model_is_user_set() const noexcept
    {
        return m_contact_default_model_user_set;
    }

  private:
    bool m_is_valid = true;
    // Diff Copy Scene Config:
    S<geometry::AttributeCollectionCommit> m_config;
    // Fully Copy:
    ObjectCollectionSnapshot m_object_collection;
    vector<ContactElement>   m_contact_elements;
    vector<SubsceneElement>  m_subscene_elements;

    // Full Copy Geometries/ Diff Copy AttributeCollection
    unordered_map<IndexT, S<geometry::GeometryCommit>> m_geometries;
    unordered_map<IndexT, S<geometry::GeometryCommit>> m_rest_geometries;
    vector<IndexT>                                     m_removed_geometry_ids;
    vector<IndexT> m_removed_rest_geometry_ids;
    IndexT         m_geometry_next_id      = -1;
    IndexT         m_rest_geometry_next_id = -1;

    // Diff Copy AttributeCollection
    S<geometry::AttributeCollectionCommit> m_contact_models;
    S<geometry::AttributeCollectionCommit> m_subscene_models;
    bool m_contact_default_model_user_set = false;
};

SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst,
                                            const SceneSnapshot& src);
}  // namespace uipc::core
