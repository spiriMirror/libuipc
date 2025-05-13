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
//          objects:
//          [
//              OBJECT Json
//          ]
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
                    slots_json.push_back(slot_json);
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
            contact_tabular["contact_elements"] =
                scene.contact_tabular().contact_elements();

            // constitution_tabular
            // don't need to store constitution tabular
            // this will be recovered from geometries

            // objects
            auto& objects_json = data["objects"];
            auto& objects      = scene.object_collection().objects();
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

    S<Scene> from_json(const Json& j)
    {
        S<Scene> scene = nullptr;

        do
        {
            auto meta_it = j.find(builtin::__meta__);
            if(meta_it == j.end())
            {
                UIPC_WARN_WITH_LOCATION("Can not find __meta__ in json");
                break;
            }
            auto& meta = *meta_it;
            if(meta["type"] != UIPC_TO_STRING(Scene))
            {
                UIPC_WARN_WITH_LOCATION("Invalid type in __meta__, expected `Scene`");
                break;
            }

            auto data_it = j.find(builtin::__data__);
            if(data_it == j.end())
            {
                UIPC_WARN_WITH_LOCATION("Can not find __data__ in json");
                break;
            }
            auto& data = *data_it;
            if(data.find("config") == data.end())
            {
                UIPC_WARN_WITH_LOCATION("Can not find `config` in __data__");
                break;
            }

            // 1) Create scene
            {
                auto config = Scene::default_config();

                // merge default config with the one in json
                // if same key, json config will override default config
                config.merge_patch(data["config"]);

                scene = std::make_shared<Scene>(config);
            }

            // 2) Build geometry atlas
            GeometryAtlas ga;
            {
                auto& geometry_atlas_json = data["geometry_atlas"];
                ga.from_json(geometry_atlas_json);
            }

            // 3) Retrive objects
            {
                auto& objects_json = data["objects"];
                if(objects_json.is_array())
                {
                    vector<S<Object>> objects;
                    objects.reserve(objects_json.size());
                    for(auto& obj_json : objects_json)
                    {
                        S<Object> object = std::make_shared<Object>();
                        object->scene(*scene);
                        uipc::core::from_json(obj_json, *object);
                        objects.push_back(object);
                    }
                    scene->object_collection().build_from(objects);
                }
                else
                {
                    UIPC_WARN_WITH_LOCATION("`objects` is not an array");
                }
            }

            // 2) Retrieve contact tabular
            {
                auto& contact_tabular = data["contact_tabular"];
                vector<ContactElement> ce;
                auto element_it = contact_tabular.find("contact_elements");
                if(element_it != contact_tabular.end())
                {
                    auto& elements = *element_it;
                    if(elements.is_array())
                    {
                        ce = elements.get<vector<ContactElement>>();
                    }
                    else
                    {
                        UIPC_WARN_WITH_LOCATION("contact_elements is not an array");
                    }
                }
                else
                {
                    UIPC_WARN_WITH_LOCATION("Can not find `contact_elements` in contact_tabular");
                }

                auto contact_models = ga.find("contact_models");
                if(contact_models && !ce.empty())
                {
                    scene->contact_tabular().build_from(*contact_models, ce);
                }
            }

            // 3) Recover geometry slots & rest geometry slots
            {
                auto& geometry_slots_json      = data["geometry_slots"];
                auto& rest_geometry_slots_json = data["rest_geometry_slots"];

                auto build_from_geo_slots =
                    [&ga](const Json& slots_json, geometry::GeometryCollection& gc)
                {
                    vector<S<geometry::GeometrySlot>> slots;
                    slots.reserve(slots_json.size());
                    for(auto& slot_json : slots_json)
                    {
                        auto id            = slot_json["id"].get<IndexT>();
                        auto index         = slot_json["index"].get<IndexT>();
                        auto geometry_slot = ga.find(index);
                        UIPC_ASSERT(geometry_slot,
                                    "Geometry slot with id {} not found in geometry atlas",
                                    index);

                        auto this_geo_slot = geometry_slot->clone();
                        this_geo_slot->id(id);
                        slots.push_back(this_geo_slot);
                    }
                    gc.build_from(slots);
                };

                auto& geometry_collection = scene->geometry_collection();
                auto& rest_geometry_collection = scene->rest_geometry_collection();

                build_from_geo_slots(geometry_slots_json, geometry_collection);
                build_from_geo_slots(rest_geometry_slots_json, rest_geometry_collection);
            }

        } while(0);

        if(!scene)
        {
            UIPC_WARN_WITH_LOCATION("Failed to create scene from json");
        }

        return scene;
    }
};

SceneFactory::SceneFactory()
    : m_impl(make_unique<Impl>())
{
}

SceneFactory::~SceneFactory() = default;

S<Scene> SceneFactory::from_json(const Json& j)
{
    return m_impl->from_json(j);
}

Json SceneFactory::to_json(const Scene& scene)
{
    return m_impl->to_json(scene);
}
}  // namespace uipc::core
