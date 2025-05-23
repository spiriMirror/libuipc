#include <uipc/core/scene_factory.h>
#include <uipc/builtin/factory_keyword.h>
#include <uipc/geometry/geometry_atlas.h>
#include <uipc/common/macro.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/geometry/geometry_commit.h>
#include <uipc/geometry/geometry_factory.h>
#include <uipc/core/internal/scene.h>
#include <uipc/common/zip.h>

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
    using GeometryAtlas       = uipc::geometry::GeometryAtlas;
    using GeometryAtlasCommit = uipc::geometry::GeometryAtlasCommit;

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
            ga.create("contact_models", *snapshot.m_contact_models);
        }

        data["geometry_atlas"] = ga.to_json();
    }

    Json to_json(const SceneSnapshot& snapshot)
    {
        Json          j = Json::object();
        GeometryAtlas ga;

        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(SceneSnapshot);
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
            auto& objects_json = data["object_collection"];
            uipc::core::to_json(objects_json, snapshot.m_object_collection);

            // - geometry slots
            // - rest geometry slots
            // - contact models
            //
            // - geometry atlas
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
        if(meta["type"] != UIPC_TO_STRING(SceneSnapshot))
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
                snapshot.m_contact_models =
                    uipc::make_shared<geometry::AttributeCollection>(*contact_models);
                snapshot.m_contact_elements = ce;
            }
        }

        // 4) Retrieve objects
        {
            auto& objects_json = data["object_collection"];
            uipc::core::from_json(objects_json, snapshot.m_object_collection);
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
        scene.contact_tabular().build_from(*snapshot.m_contact_models,
                                           snapshot.m_contact_elements);


        // 3) Geometry
        // 4) Rest Geometry
        {
            auto build_geometries =
                [&gf, &scene](const unordered_map<IndexT, S<geometry::Geometry>>& geometries,
                              geometry::GeometryCollection& geometry_collection)
            {
                vector<S<geometry::GeometrySlot>> geometry_slots;
                geometry_slots.reserve(geometries.size());
                for(auto&& [id, geo] : geometries)
                {
                    geometry_slots.push_back(gf.create_slot(id, *geo));
                }
                geometry_collection.build_from(geometry_slots);
            };

            build_geometries(snapshot.m_geometries, scene.m_internal->geometries());

            build_geometries(snapshot.m_rest_geometries,
                             scene.m_internal->rest_geometries());
        }

        // 5) Objects
        auto&             objects = snapshot.m_object_collection.m_objects;
        vector<S<Object>> object_slots;
        object_slots.reserve(objects.size());
        for(auto&& [id, obj] : objects)
        {
            auto object =
                uipc::make_shared<Object>(*scene.m_internal, obj.m_id, obj.m_name);
            object->build_from(obj.m_geometries);
            object_slots.push_back(std::move(object));
        }
        scene.m_internal->objects().build_from(object_slots);

        return scene;
    }

    Json commit_to_json(const SceneSnapshotCommit& commit)
    {
        Json                j = Json::object();
        GeometryAtlasCommit gac;

        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(SceneSnapshotCommit);
        }
        auto& data = j[builtin::__data__];
        {
            data["config"] = commit.m_config;
            // contact_tabular
            auto& contact_tabular               = data["contact_tabular"];
            contact_tabular["contact_elements"] = commit.m_contact_elements;
            // constitution_tabular
            // don't need to store constitution tabular
            // this will be recovered from geometries
            // objects
            auto& objects_json = data["object_collection"];
            uipc::core::to_json(objects_json, commit.m_object_collection);

            gac.create("contact_models", *commit.m_contact_models);

            // geometry slots
            auto& geo_slots_json      = data["geometry_slots"];
            auto& rest_geo_slots_json = data["rest_geometry_slots"];

            auto setup = [&](Json& slots_json,
                             const unordered_map<IndexT, S<geometry::GeometryCommit>>& geos)
            {
                auto& geo_commit = commit.m_geometries;
                for(auto& [id, geo] : geos)
                {
                    Json slot_json     = Json::object();
                    slot_json["id"]    = id;
                    slot_json["index"] = gac.create(*geo);
                    slots_json.push_back(slot_json);
                }
            };

            setup(geo_slots_json, commit.m_geometries);
            setup(rest_geo_slots_json, commit.m_rest_geometries);

            // geometry atlas
            data["geometry_atlas"] = gac.to_json();
        }
        return j;
    }

    SceneSnapshotCommit commit_from_json(const Json& json)
    {
        SceneSnapshotCommit commit;
        GeometryAtlasCommit gac;
        try
        {
            do
            {
                auto meta_it = json.find(builtin::__meta__);
                if(meta_it == json.end())
                {
                    UIPC_WARN_WITH_LOCATION("Can not find __meta__ in json");
                    commit.m_is_valid = false;
                    break;
                }

                auto& meta = *meta_it;
                if(meta["type"] != UIPC_TO_STRING(SceneSnapshotCommit))
                {
                    UIPC_WARN_WITH_LOCATION("Invalid type in __meta__, expected `SceneSnapshotCommit`");
                    commit.m_is_valid = false;
                    break;
                }

                auto data_it = json.find(builtin::__data__);
                if(data_it == json.end())
                {
                    UIPC_WARN_WITH_LOCATION("Can not find __data__ in json");
                    commit.m_is_valid = false;
                    break;
                }

                auto& data = *data_it;

                // 1) Config
                {
                    auto config_it = data.find("config");
                    if(config_it == data.end())
                    {
                        UIPC_WARN_WITH_LOCATION("Can not find `config` in json");
                        commit.m_is_valid = false;
                        break;
                    }

                    auto config = Scene::default_config();
                    // merge default config with the one in json
                    // if same key, json config will override default config

                    config.merge_patch(data["config"]);
                    commit.m_config = config;
                }

                // 2) Build geometry atlas commit
                {
                    auto geometry_atlas_it = data.find("geometry_atlas");
                    if(geometry_atlas_it == data.end())
                    {
                        UIPC_WARN_WITH_LOCATION("Can not find `geometry_atlas` in json");
                        commit.m_is_valid = false;
                        break;
                    }

                    auto& geometry_atlas_json = *geometry_atlas_it;
                    gac.from_json(geometry_atlas_json);
                }


                // 3) Build object collection commit
                {
                    auto object_collection_it = data.find("object_collection");
                    if(object_collection_it == data.end())
                    {
                        UIPC_WARN_WITH_LOCATION("Can not find `object_collection` in json");
                        commit.m_is_valid = false;
                        break;
                    }
                    auto& objects_json = *object_collection_it;
                    uipc::core::from_json(objects_json, commit.m_object_collection);
                }

                // 4) Retrieve contact tabular
                {
                    auto contact_tabular_it = data.find("contact_tabular");
                    if(contact_tabular_it == data.end())
                    {
                        UIPC_WARN_WITH_LOCATION("Can not find `contact_tabular` in json");
                        commit.m_is_valid = false;
                        break;
                    }

                    auto& contact_tabular = *contact_tabular_it;
                    auto contact_element_it = contact_tabular.find("contact_elements");
                    if(contact_element_it == contact_tabular.end())
                    {
                        UIPC_WARN_WITH_LOCATION("Can not find `contact_elements` in contact_tabular");
                        commit.m_is_valid = false;
                        break;
                    }

                    auto& contact_element = *contact_element_it;
                    commit.m_contact_elements =
                        contact_element.get<vector<ContactElement>>();
                    auto contact_models = gac.find("contact_models");
                    if(!contact_models)
                    {
                        UIPC_WARN_WITH_LOCATION("Contact models not found in geometry atlas");
                        commit.m_is_valid = false;
                        break;
                    }
                    commit.m_contact_models =
                        uipc::make_shared<geometry::AttributeCollectionCommit>(*contact_models);
                }


                // 5) Recover geometry slots & rest geometry slots
                auto& geometry_slots_json      = data["geometry_slots"];
                auto& rest_geometry_slots_json = data["rest_geometry_slots"];

                auto build_geo_commits =
                    [&gac](const Json& commits_json,
                           unordered_map<IndexT, S<geometry::GeometryCommit>>& geo_commits)
                {
                    for(auto& slot_json : commits_json)
                    {
                        IndexT id    = slot_json["id"].get<IndexT>();
                        IndexT index = slot_json["index"].get<IndexT>();
                        auto   geometry_commit = gac.find(index);
                        if(geometry_commit == nullptr)
                        {
                            UIPC_WARN_WITH_LOCATION("Geometry commit with id {} not found in geometry atlas",
                                                    index);
                            continue;
                        }

                        geo_commits[id] =
                            uipc::make_shared<geometry::GeometryCommit>(*geometry_commit);
                    }
                };

                build_geo_commits(geometry_slots_json, commit.m_geometries);
                build_geo_commits(rest_geometry_slots_json, commit.m_rest_geometries);

                if(commit.m_geometries.size() != commit.m_rest_geometries.size())
                {
                    UIPC_WARN_WITH_LOCATION("Geometry commit size does not match rest geometry commit size");
                    commit.m_is_valid = false;
                    break;
                }

                for(auto&& [geo, rest_geo] : zip(commit.m_geometries, commit.m_rest_geometries))
                {
                    if(geo.first != rest_geo.first)
                    {
                        UIPC_WARN_WITH_LOCATION(
                            "Geometry commit id does not match rest geometry commit id, "
                            "geo_id = {}, rest_geo_id = {}",
                            geo.first,
                            rest_geo.first);
                        commit.m_is_valid = false;
                        break;
                    }
                }

            } while(0);
        }
        catch(const std::exception& e)
        {
            UIPC_WARN_WITH_LOCATION("Failed to parse SceneSnapshotCommit from json: {}",
                                    e.what());
            commit.m_is_valid = false;
        }

        return commit;
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

SceneSnapshotCommit SceneFactory::commit_from_json(const Json& json)
{
    return m_impl->commit_from_json(json);
}

Json SceneFactory::commit_to_json(const SceneSnapshotCommit& scene)
{
    return m_impl->commit_to_json(scene);
}
}  // namespace uipc::core
