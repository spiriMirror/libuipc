#include <uipc/geometry/geometry_factory.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/implicit_geometry.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/attribute_collection_factory.h>
#include <uipc/builtin/factory_keyword.h>
#include <uipc/common/zip.h>
#include <uipc/common/macro.h>

#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/simplicial_complex_slot.h>
#include <uipc/geometry/implicit_geometry_slot.h>


// A Json representation of the geometry may look like this:
//  {
//      __meta__:
//      {
//            base: "Geometry",,
//            type: "SimplicialComplex"
//      },
//      __data__:
//      {
//            // AttributeCollection json representation
//            meta:{ ... },
//            instances: { ... },
//            vertices: {...},
//            edges: {...},
//            triangles: {...},
//            tetrahedra: {...}
//      }
//  }

namespace uipc::geometry
{
using Creator = std::function<S<Geometry>(const Json&, span<S<IAttributeSlot>>)>;
using GeometryToSlot = std::function<S<GeometrySlot>(IndexT, const Geometry&)>;

template <>
class GeometryFriend<GeometryFactory>
{
  public:
    static void build_from_attribute_collections(Geometry&         geometry,
                                                 span<std::string> names,
                                                 span<S<AttributeCollection>> collections) noexcept
    {
        vector<AttributeCollection*> collections_ptr;
        collections_ptr.reserve(collections.size());
        std::transform(collections.begin(),
                       collections.end(),
                       std::back_inserter(collections_ptr),
                       [](const S<AttributeCollection>& ac) { return ac.get(); });

        geometry.do_build_from_attribute_collections(names, collections_ptr);
    }

    static void collect_attribute_collections(Geometry&            geometry,
                                              vector<std::string>& names,
                                              vector<AttributeCollection*>& collections) noexcept
    {
        geometry.collect_attribute_collections(names, collections);
    }
};


template <std::derived_from<Geometry> T>
static void register_geometry_from_json(AttributeCollectionFactory& acf,
                                        std::unordered_map<std::string, Creator>& creators,
                                        std::string_view type_name)
{
    using GF = GeometryFriend<GeometryFactory>;
    creators.insert(
        {std::string{type_name},
         [&](const Json& j, span<S<IAttributeSlot>> attributes) -> S<Geometry>
         {
             S<T> geometry = std::make_shared<T>();

             vector<std::string> names;
             names.reserve(8);
             vector<S<AttributeCollection>> collections;
             collections.reserve(8);

             for(auto&& [k, ac_json] : j.items())
             {
                 names.push_back(k);
                 S<AttributeCollection> ac = acf.from_json(ac_json, attributes);
                 collections.push_back(ac);
             }

             GF::build_from_attribute_collections(*geometry, names, collections);

             return geometry;
         }});
}

template <std::derived_from<Geometry> T>
static void register_slot_from_geometry(std::unordered_map<std::string, GeometryToSlot>& creators,
                                        std::string_view type_name)
{
    creators.insert({std::string{type_name},
                     [&](IndexT id, const Geometry& geometry) -> S<GeometrySlot>
                     {
                         auto slot =
                             std::make_shared<GeometrySlotT<T>>(id, *geometry.as<T>());
                         return std::static_pointer_cast<GeometrySlot>(slot);
                     }});
}


class GeometryFactory::Impl
{
  public:
    static auto& acf()
    {
        static thread_local AttributeCollectionFactory acf;
        return acf;
    }

    static auto& creators()
    {
        static thread_local std::once_flag                           f;
        static thread_local std::unordered_map<std::string, Creator> m_creators;

        std::call_once(
            f,
            [&]
            {
#define UIPC_GEOMETRY_EXPORT_DEF(T)                                            \
    register_geometry_from_json<T>(acf(), m_creators, UIPC_TO_STRING(T));

        // This is a macro to register all visible geometry types
#include <uipc/geometry/details/geometry_export_types.inl>

#undef UIPC_GEOMETRY_EXPORT_DEF
            });


        return m_creators;
    }

    static auto& slot_creators()
    {
        static thread_local std::once_flag f;
        static thread_local std::unordered_map<std::string, GeometryToSlot> m_creators;
        std::call_once(f,
                       [&]
                       {
#define UIPC_GEOMETRY_EXPORT_DEF(T)                                            \
    register_slot_from_geometry<T>(m_creators, UIPC_TO_STRING(T));

        // This is a macro to register all visible geometry types
#include <uipc/geometry/details/geometry_export_types.inl>

#undef UIPC_GEOMETRY_EXPORT_DEF
                       });
        return m_creators;
    }

    S<Geometry> geometry_from_json(const Json& json, span<S<IAttributeSlot>> attributes)
    {
        auto meta_it = json.find(builtin::__meta__);
        if(meta_it == json.end())
        {
            UIPC_WARN_WITH_LOCATION("`__meta__` info not found, so we ignore it");
            return nullptr;
        }
        auto& meta    = *meta_it;
        auto  type_it = meta.find("type");
        if(type_it == meta.end())
        {
            UIPC_WARN_WITH_LOCATION("`__meta__.type` not found, so we ignore it");
            return nullptr;
        }
        auto type = type_it->get<std::string>();

        auto base_it = meta.find("base");
        if(base_it == meta.end())
        {
            UIPC_WARN_WITH_LOCATION("`__meta__.base` not found, so we ignore it");
            return nullptr;
        }
        auto& base = *base_it;
        if(base.get<std::string>() != UIPC_TO_STRING(Geometry))
        {
            UIPC_WARN_WITH_LOCATION("`__meta__.base` not match, so we ignore it");
            return nullptr;
        }

        auto creator_it = creators().find(type);
        if(creator_it == creators().end())
        {
            UIPC_WARN_WITH_LOCATION("Geometry<{}> not registered, so we ignore it", type);
            return nullptr;
        }

        // build from data
        auto data_it = json.find(builtin::__data__);
        if(data_it == json.end())
        {
            UIPC_WARN_WITH_LOCATION("`__data__` not found in json, skip.");
            return nullptr;
        }

        auto& data     = *data_it;
        auto  geometry = creator_it->second(data, attributes);

        return geometry;
    }

    vector<S<Geometry>> from_json(const Json& j, span<S<IAttributeSlot>> attributes)
    {
        UIPC_ASSERT(j.is_array(), "To create a Geometries, this json must be an array");

        vector<S<Geometry>> geometries;
        geometries.reserve(j.size());
        for(SizeT i = 0; i < j.size(); ++i)
        {
            auto& items    = j[i];
            auto  geometry = geometry_from_json(items, attributes);
            if(geometry)
            {
                geometries.push_back(geometry);
            }
        }

        return geometries;
    }


    Json geometry_to_json(Geometry& geometry, unordered_map<IAttribute*, IndexT> attr_to_index)
    {
        using GF   = GeometryFriend<GeometryFactory>;
        Json  j    = Json::object();
        auto& meta = j[builtin::__meta__];
        {
            meta["base"] = "Geometry";
            meta["type"] = geometry.type();
        }
        auto& data = j[builtin::__data__];
        {
            vector<std::string> names;
            names.reserve(8);
            vector<AttributeCollection*> collections;
            collections.reserve(8);
            GF::collect_attribute_collections(geometry, names, collections);
            for(auto&& [name, collection] : zip(names, collections))
            {
                data[name] = acf().to_json(collection, attr_to_index);
            }
        }
        return j;
    }

    Json to_json(span<Geometry*> geos, unordered_map<IAttribute*, IndexT> attr_to_index)
    {
        using GF = GeometryFriend<GeometryFactory>;
        Json j   = Json::array();

        for(auto&& geo : geos)
        {
            j.push_back(geometry_to_json(*geo, attr_to_index));
        }

        return j;
    }

    S<GeometrySlot> create_slot(IndexT id, const Geometry& geometry)
    {
        auto type = geometry.type();
        auto it   = slot_creators().find(std::string{type});
        UIPC_ASSERT(it != slot_creators().end(),
                    "Geometry<{}> not registered, why can it happen?",
                    type);
        auto creator = it->second;
        return creator(id, geometry);
    }
};


GeometryFactory::GeometryFactory()
    : m_impl{uipc::make_unique<Impl>()}
{
}

GeometryFactory::~GeometryFactory() {}

vector<S<Geometry>> GeometryFactory::from_json(const Json& j, span<S<IAttributeSlot>> geos)
{
    return m_impl->from_json(j, geos);
}

S<GeometrySlot> GeometryFactory::create_slot(IndexT id, const Geometry& geometry)
{
    return m_impl->create_slot(id, geometry);
}

Json GeometryFactory::to_json(span<Geometry*> geos, unordered_map<IAttribute*, IndexT> attr_to_index)
{
    return m_impl->to_json(geos, attr_to_index);
}
}  // namespace uipc::geometry
