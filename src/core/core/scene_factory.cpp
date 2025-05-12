#include <uipc/core/scene_factory.h>
#include <uipc/builtin/factory_keyword.h>
#include <uipc/geometry/geometry_atlas.h>
#include <uipc/common/macro.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/geometry/geometry_atlas.h>

namespace uipc::core
{
// A Json representation of a Scene may look like this:
//
//  {
//      __meta__:
//      {
//          type:"Scene"
//      },
//      __data__:
//      {
//          config:{ ... },
//
//          contact_tabular:
//          {
//              elements:{}
//          }
//
//          geometry_atlas:
//          {
//              ATLAS Json
//          }
//     }
//  }


class SceneFactory::Impl
{
  public:
    Impl()  = default;
    ~Impl() = default;

    Scene from_json(const Json& j) {}

    using GeometryAtlas = uipc::geometry::GeometryAtlas;

    void build_geometry_atlas_from_scene(const Scene& scene, Json& data, GeometryAtlas& ga)
    {
        // geometries
        {
            auto setup = [&](Json& slots_json, span<S<uipc::geometry::GeometrySlot>> slots)
            {
                auto geo_slots = scene.geometry_collection().geometry_slots();
                slots_json     = Json::array();
                for(auto& slot : slots)
                {
                    Json slot_json     = Json::object();
                    slot_json["id"]    = slot->id();
                    slot_json["index"] = ga.create(slot->geometry());
                }
            };

            // geometry slots
            auto& geo_slots_json = data["geometry_slots"];
            auto  geo_slots      = scene.geometry_collection().geometry_slots();
            setup(geo_slots_json, geo_slots);

            // rest geometry slots
            auto& rest_geo_slots_json = data["rest_geometry_slots"];
            auto rest_geo_slots = scene.rest_geometry_collection().geometry_slots();
            setup(rest_geo_slots_json, rest_geo_slots);
        }

        // contact models
        {
            ga.create("contact_models", scene.contact_tabular().internal_contact_models());
        }

        data["geometry_atlas"] = ga.to_json();
    }


    Json to_json(const Scene& scene)
    {
        Json          j;
        GeometryAtlas ga;

        // __meta__
        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(Scene);
        }

        // __data__
        auto& data = j[builtin::__data__];
        {
            data["config"] = scene.config();

            // contact_tabular
            auto& contact_tabular = data["contact_tabular"];
            contact_tabular["elements"] = scene.contact_tabular().contact_elements();

            // objects
            auto& objects_json = data["objects"];
            auto& objects      = scene.object_collection().m_objects;
            objects_json       = Json::array();
            for(auto&& [id, object] : objects)
            {
                objects_json.push_back(*object);
            }

            // geometry_atlas
            auto& geometry_atlas = data["geometry_atlas"];
            build_geometry_atlas_from_scene(scene, data, ga);
        }

        return j;
    }
};

SceneFactory::SceneFactory()
    : m_impl(make_unique<Impl>())
{
}

SceneFactory::~SceneFactory() = default;

Scene SceneFactory::from_json(const Json& j)
{
    return m_impl->from_json(j);
}

Json SceneFactory::to_json(const Scene& scene)
{
    return m_impl->to_json(scene);
}


}  // namespace uipc::core
