#include <uipc/core/scene_factory.h>
#include <uipc/builtin/factory_keyword.h>
#include <uipc/geometry/geometry_atlas.h>
#include <uipc/common/macro.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/geometry/geometry_commit.h>
#include <uipc/geometry/geometry_factory.h>
#include <uipc/core/internal/scene.h>

namespace uipc::core
{
// A Json representation of a Scene may look like this:
//
//  {
//      __meta__:
//      {
//          type:"Scene"
//          kind:"init"/"commit"
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

    void build_geometry_atlas_from_scene_snapshot(const SceneSnapshot& snapshot,
                                                  Json&                data,
                                                  GeometryAtlas&       ga)
    {
        // geometries
        {
            auto setup = [&](Json& slots_json,
                             const unordered_map<IndexT, S<geometry::Geometry>>& geos)
            {
                auto geo_slots = snapshot.m_geometries;
                slots_json     = Json::array();
                for(auto& [id, geo] : geos)
                {
                    Json slot_json     = Json::object();
                    slot_json["id"]    = id;
                    slot_json["index"] = ga.create(*geo);
                    slots_json.push_back(slot_json);
                }
            };

            // geometry slots
            auto& geo_slots_json = data["geometry_slots"];
            setup(geo_slots_json, snapshot.m_geometries);

            // rest geometry slots
            auto& rest_geo_slots_json = data["rest_geometry_slots"];
            setup(rest_geo_slots_json, snapshot.m_rest_geometries);
        }

        // contact models
        {
            ga.create("contact_models", snapshot.m_contact_models);
        }

        data["geometry_atlas"] = ga.to_json();
    }

    Json to_json(const SceneSnapshot& snapshot)
    {
        Json          j = Json::object();
        GeometryAtlas ga;

        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(Scene);
        }
        auto& data = j[builtin::__data__];
        {
            data["config"] = snapshot.m_config;
            // contact_tabular
            auto& contact_tabular               = data["contact_tabular"];
            contact_tabular["contact_elements"] = snapshot.m_contact_elements;
            // constitution_tabular
            // don't need to store constitution tabular
            // this will be recovered from geometries
            // objects
            auto& objects_json = data["objects"];
            auto& objects      = snapshot.m_objects;
            objects_json       = Json::array();
            for(auto&& obj : objects)
            {
                Json obj_json;
                obj_json["id"]         = obj.m_id;
                obj_json["name"]       = obj.m_name;
                obj_json["geometries"] = obj.m_geometries;
                objects_json.push_back(obj_json);
            }
            // geometry_atlas
            auto& geometry_atlas = data["geometry_atlas"];
            build_geometry_atlas_from_scene_snapshot(snapshot, data, ga);
        }
        return j;
    }

    SceneSnapshot from_json(const Json& j)
    {
        SceneSnapshot snapshot;
        auto          meta_it = j.find(builtin::__meta__);
        if(meta_it == j.end())
        {
            UIPC_WARN_WITH_LOCATION("Can not find __meta__ in json");
            return snapshot;
        }
        auto& meta = *meta_it;
        if(meta["type"] != UIPC_TO_STRING(Scene))
        {
            UIPC_WARN_WITH_LOCATION("Invalid type in __meta__, expected `Scene`");
            return snapshot;
        }
        auto data_it = j.find(builtin::__data__);
        if(data_it == j.end())
        {
            UIPC_WARN_WITH_LOCATION("Can not find __data__ in json");
            return snapshot;
        }
        auto& data = *data_it;

        // 1) Config
        {
            auto config = Scene::default_config();
            // merge default config with the one in json
            // if same key, json config will override default config
            config.merge_patch(data["config"]);
            snapshot.m_config = config;
        }

        // 2) Build geometry atlas
        GeometryAtlas ga;
        {
            auto& geometry_atlas_json = data["geometry_atlas"];
            ga.from_json(geometry_atlas_json);
        }

        // 3) Retrieve contact tabular
        {
            auto&                  contact_tabular = data["contact_tabular"];
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

            // contact models
            auto contact_models = ga.find("contact_models");
            if(contact_models && !ce.empty())
            {
                snapshot.m_contact_models   = *contact_models;
                snapshot.m_contact_elements = ce;
            }
        }

        // 4) Retrieve objects
        {
            auto& objects_json = data["objects"];
            if(objects_json.is_array())
            {
                snapshot.m_objects.reserve(objects_json.size());
                for(auto& obj_json : objects_json)
                {
                    ObjectSnapshot snap_obj;
                    snap_obj.m_id   = obj_json["id"].get<IndexT>();
                    snap_obj.m_name = obj_json["name"].get<std::string>();
                    auto ids = obj_json["geometries"].get<vector<IndexT>>();
                    snap_obj.m_geometries = ids;
                    snapshot.m_objects.push_back(snap_obj);
                }
            }
            else
            {
                UIPC_WARN_WITH_LOCATION("`objects` is not an array");
            }
        }

        // 5) Recover geometry slots & rest geometry slots
        {
            auto& geometry_slots_json      = data["geometry_slots"];
            auto& rest_geometry_slots_json = data["rest_geometry_slots"];

            auto build_from_geo_slots =
                [&ga](const Json&                                   slots_json,
                      unordered_map<IndexT, S<geometry::Geometry>>& geos)
            {
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
                    geos[id] = std::static_pointer_cast<geometry::Geometry>(
                        this_geo_slot->geometry().clone());
                }
            };

            auto& geometry_collection      = snapshot.m_geometries;
            auto& rest_geometry_collection = snapshot.m_rest_geometries;

            build_from_geo_slots(geometry_slots_json, geometry_collection);
            build_from_geo_slots(rest_geometry_slots_json, rest_geometry_collection);
        }

        return snapshot;
    }

    Scene from_snapshot(const SceneSnapshot& snapshot)
    {
        geometry::GeometryFactory gf;

        // 1) Config
        Scene scene{snapshot.m_config};
        // 2) Contact tabular
        scene.contact_tabular().build_from(snapshot.m_contact_models, snapshot.m_contact_elements);

        // 3) Geometries
        vector<S<geometry::GeometrySlot>> geometry_slots;
        geometry_slots.reserve(snapshot.m_geometries.size());
        for(auto&& [id, geo] : snapshot.m_geometries)
        {
            geometry_slots.push_back(gf.create_slot(id, *geo));
        }
        scene.m_internal->geometries().build_from(geometry_slots);

        // 4) Rest Geometries
        vector<S<geometry::GeometrySlot>> rest_geometry_slots;
        rest_geometry_slots.reserve(snapshot.m_rest_geometries.size());
        for(auto&& [id, geo] : snapshot.m_rest_geometries)
        {
            rest_geometry_slots.push_back(gf.create_slot(id, *geo));
        }
        scene.m_internal->geometries().build_from(rest_geometry_slots);

        // 5) Objects
        auto&             objects = snapshot.m_objects;
        vector<S<Object>> object_slots;
        object_slots.reserve(objects.size());
        for(auto&& obj : objects)
        {
            auto object = std::make_unique<Object>(*scene.m_internal, obj.m_id, obj.m_name);
            object->build_from(obj.m_geometries);
        }
        scene.m_internal->objects().build_from(object_slots);

        return scene;
    }
};

SceneFactory::SceneFactory()
    : m_impl(uipc::make_unique<Impl>())
{
}

SceneFactory::~SceneFactory() = default;

Scene SceneFactory::from_snapshot(const SceneSnapshot& snapshot)
{
    return m_impl->from_snapshot(snapshot);
}

SceneSnapshot SceneFactory::from_json(const Json& j)
{
    return m_impl->from_json(j);
}

Json SceneFactory::to_json(const SceneSnapshot& scene)
{
    return m_impl->to_json(scene);
}
}  // namespace uipc::core
