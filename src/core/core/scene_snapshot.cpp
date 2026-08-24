#include <uipc/core/scene_snapshot.h>
#include <uipc/core/internal/scene.h>
#include <uipc/geometry/geometry_commit.h>
#include <algorithm>

namespace uipc::core
{
SceneSnapshot::SceneSnapshot(const Scene& scene)
{
    m_contact_models = uipc::make_shared<geometry::AttributeCollection>(
        scene.contact_tabular().internal_contact_models());

    m_subscene_models = uipc::make_shared<geometry::AttributeCollection>(
        scene.subscene_tabular().internal_subscene_models());

    m_contact_default_model_user_set =
        scene.contact_tabular().default_model_is_user_set();

    m_config =
        uipc::make_shared<geometry::AttributeCollection>(scene.m_internal->config());

    auto& internal_scene = *scene.m_internal;
    UIPC_ASSERT_THROW(
        internal_scene.geometries().pending_create_slots().size() == 0
            && internal_scene.rest_geometries().pending_create_slots().size() == 0
            && internal_scene.geometries().pending_destroy_ids().size() == 0
            && internal_scene.rest_geometries().pending_destroy_ids().size() == 0,
        R"(GeometryCollection has pending create slots, you should create SceneSnapshot immediately after:
- world.init()
- world.advance()
)");

    m_geometry_next_id      = internal_scene.geometries().next_id();
    m_rest_geometry_next_id = internal_scene.rest_geometries().next_id();
    UIPC_ASSERT_THROW(m_geometry_next_id == m_rest_geometry_next_id,
                      "Geometry and rest geometry next ids do not match ({} != {}).",
                      m_geometry_next_id,
                      m_rest_geometry_next_id);

    // retrieve contact elements
    {
        auto span = scene.contact_tabular().contact_elements();
        m_contact_elements.resize(span.size());
        std::ranges::copy(span, m_contact_elements.begin());
    }
    // retrieve subscene elements
    {
        auto span = scene.subscene_tabular().subscene_elements();
        m_subscene_elements.resize(span.size());
        std::ranges::copy(span, m_subscene_elements.begin());
    }

    // retrieve constitution elements
    auto& objects       = internal_scene.objects();
    m_object_collection = ObjectCollectionSnapshot{objects};

    // retrieve geometries
    auto geometry_slots = internal_scene.geometries().geometry_slots();
    m_geometries.reserve(geometry_slots.size());
    for(auto&& slot : geometry_slots)
    {
        auto& geometry = slot->geometry();
        m_geometries[slot->id()] =
            std::static_pointer_cast<geometry::Geometry>(geometry.clone());
    }

    // retrieve rest geometries
    auto rest_geometry_slots = internal_scene.rest_geometries().geometry_slots();
    m_rest_geometries.reserve(rest_geometry_slots.size());
    for(auto&& slot : rest_geometry_slots)
    {
        auto& geometry = slot->geometry();
        m_rest_geometries[slot->id()] =
            std::static_pointer_cast<geometry::Geometry>(geometry.clone());
    }

    UIPC_ASSERT_THROW(m_geometries.size() == m_rest_geometries.size(),
                      "Geometry and rest geometry counts do not match ({} != {}).",
                      m_geometries.size(),
                      m_rest_geometries.size());
    for(auto&& [id, _] : m_geometries)
    {
        UIPC_ASSERT_THROW(m_rest_geometries.contains(id), "Rest geometry with id {} is missing.", id);
    }
}

SceneSnapshotCommit::SceneSnapshotCommit(const SceneSnapshot& dst, const SceneSnapshot& src)
    : m_config{uipc::make_shared<geometry::AttributeCollectionCommit>(
          *dst.m_config - *src.m_config)}
    , m_geometry_next_id{dst.m_geometry_next_id}
    , m_rest_geometry_next_id{dst.m_rest_geometry_next_id}
    , m_contact_models{uipc::make_shared<geometry::AttributeCollectionCommit>(
          *dst.m_contact_models - *src.m_contact_models)}
    , m_subscene_models{uipc::make_shared<geometry::AttributeCollectionCommit>(
          *dst.m_subscene_models - *src.m_subscene_models)}
    , m_contact_default_model_user_set{dst.m_contact_default_model_user_set}
{
    m_object_collection = dst.m_object_collection;

    m_contact_elements  = dst.m_contact_elements;
    m_subscene_elements = dst.m_subscene_elements;

    auto setup = [this](const unordered_map<IndexT, S<geometry::Geometry>>& dst_geometries,
                        const unordered_map<IndexT, S<geometry::Geometry>>& src_geometries,
                        unordered_map<IndexT, S<geometry::GeometryCommit>>& commits,
                        vector<IndexT>& removed_ids)
    {
        commits.reserve(dst_geometries.size());
        for(auto&& [id, dst_geo] : dst_geometries)
        {
            auto src_geo = src_geometries.find(id);
            if(src_geo != src_geometries.end())
            {
                commits[id] = uipc::make_shared<geometry::GeometryCommit>(
                    *dst_geo - (*src_geo->second));
            }
            else
            {
                commits[id] = uipc::make_shared<geometry::GeometryCommit>(*dst_geo);
            }

            if(!commits[id]->is_valid())
                m_is_valid = false;
        }

        removed_ids.reserve(src_geometries.size());
        for(auto&& [id, _] : src_geometries)
        {
            if(!dst_geometries.contains(id))
                removed_ids.push_back(id);
        }
        std::ranges::sort(removed_ids);
    };

    setup(dst.m_geometries, src.m_geometries, m_geometries, m_removed_geometry_ids);
    setup(dst.m_rest_geometries, src.m_rest_geometries, m_rest_geometries, m_removed_rest_geometry_ids);

    if(m_geometries.size() != m_rest_geometries.size() || m_removed_geometry_ids != m_removed_rest_geometry_ids
       || m_geometry_next_id != m_rest_geometry_next_id)
    {
        m_is_valid = false;
        UIPC_WARN_WITH_LOCATION("Geometry and rest geometry commits do not describe the same topology.");
    }

    for(auto&& [id, _] : m_geometries)
    {
        if(!m_rest_geometries.contains(id))
        {
            m_is_valid = false;
            UIPC_WARN_WITH_LOCATION("Rest geometry commit with id {} is missing.", id);
        }
    }
}

SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst, const SceneSnapshot& src)
{
    return SceneSnapshotCommit{dst, src};
}
}  // namespace uipc::core
