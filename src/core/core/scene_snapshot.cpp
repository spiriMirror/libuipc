#include <uipc/core/scene_snapshot.h>
#include <uipc/core/internal/scene.h>
#include <uipc/geometry/geometry_commit.h>

namespace uipc::core
{
SceneSnapshot::SceneSnapshot(const Scene& scene)
    : m_config(scene.config())
{
    m_contact_models = uipc::make_shared<geometry::AttributeCollection>(
        scene.contact_tabular().internal_contact_models());

    auto& internal_scene = *scene.m_internal;
    UIPC_ASSERT(internal_scene.geometries().pending_create_slots().size() == 0
                    && internal_scene.rest_geometries().pending_create_slots().size() == 0
                    && internal_scene.geometries().pending_destroy_ids().size() == 0
                    && internal_scene.rest_geometries().pending_destroy_ids().size() == 0,
                R"(GeometryCollection has pending create slots, you should create SceneSnapshot immediately after:
- world.init()
- world.advance()
)");

    // retrieve contact elements
    auto span = scene.contact_tabular().contact_elements();
    m_contact_elements.resize(span.size());
    std::ranges::copy(span, m_contact_elements.begin());

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
}

SceneSnapshotCommit::SceneSnapshotCommit(const SceneSnapshot& dst, const SceneSnapshot& src)
    : m_contact_models(uipc::make_shared<geometry::AttributeCollectionCommit>(
        *dst.m_contact_models - *src.m_contact_models))
{
    m_config            = dst.m_config;
    m_object_collection = dst.m_object_collection;
    m_contact_elements  = dst.m_contact_elements;

    auto setup = [&dst, &src](unordered_map<IndexT, S<geometry::GeometryCommit>>& gcs)
    {
        gcs.reserve(dst.m_geometries.size());
        for(auto&& [id, dst_geo] : dst.m_geometries)
        {
            auto src_geo = src.m_geometries.find(id);
            if(src_geo != src.m_geometries.end())
            {
                // diff geometry
                gcs[id] = uipc::make_shared<geometry::GeometryCommit>(
                    *dst_geo - (*src_geo->second));
            }
            else
            {
                // new geometry
                gcs[id] = uipc::make_shared<geometry::GeometryCommit>(*dst_geo);
            }
        }
    };

    // geometries
    setup(m_geometries);
    // rest geometries
    setup(m_rest_geometries);
}

SceneSnapshotCommit UIPC_CORE_API operator-(const SceneSnapshot& dst, const SceneSnapshot& src)
{
    return SceneSnapshotCommit{dst, src};
}
}  // namespace uipc::core
