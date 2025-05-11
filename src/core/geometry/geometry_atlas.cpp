#include <uipc/geometry/geometry_atlas.h>
#include <unordered_set>
#include <unordered_map>
#include <uipc/common/macro.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/span.h>
#include <uipc/geometry/attribute_factory.h>
#include <uipc/common/zip.h>

namespace uipc::geometry
{
template <>
class AttributeFriend<GeometryAtlas>
{
  public:
    static const auto& attribute_slots(const AttributeCollection& ac)
    {
        return ac.m_attributes;
    }

    static auto attribute(S<IAttributeSlot> attr_slot) -> IAttribute*
    {
        return &attr_slot->attribute();
    }
};

template <>
class GeometryFriend<GeometryAtlas>
{
  public:
    static void attribute_collections(Geometry&                     geometry,
                                      vector<std::string>&          names,
                                      vector<AttributeCollection*>& collections)
    {
        geometry.collect_attribute_collections(names, collections);
    }
};


class GeometryAtlas::Impl
{
  public:
    /**************************************************************
    *                          Building
    ***************************************************************/

    vector<S<Geometry>>                                m_geometries;
    unordered_map<std::string, S<AttributeCollection>> m_attribute_collections;
    unordered_map<IAttribute*, IndexT>                 m_attr_to_index;
    vector<IAttribute*>                                m_index_to_attr;

    IndexT create(const Geometry& geo)
    {
        S<Geometry> new_geo = std::static_pointer_cast<Geometry>(geo.clone());
        build_attributes_index_from_geometry(*new_geo);
        IndexT id        = static_cast<IndexT>(m_geometries.size());
        m_geometries[id] = std::move(new_geo);

        return id;
    }

    const Geometry* find(IndexT id) const
    {
        if(id >= m_geometries.size())
        {
            UIPC_WARN_WITH_LOCATION("Geometry with id {} not found", id);
            return nullptr;
        }
        else
        {
            return m_geometries[id].get();
        }
    }

    void create(std::string_view name, const AttributeCollection& ac)
    {
        S<AttributeCollection> new_ac = std::make_shared<AttributeCollection>(ac);
        build_attributes_index_from_attribute_collection(*new_ac);
        auto it = m_attribute_collections.find(std::string{name});
        if(it == m_attribute_collections.end())
        {
            m_attribute_collections[std::string{name}] = std::move(new_ac);
        }
        else
        {
            it->second = std::move(new_ac);
            UIPC_WARN_WITH_LOCATION("AttributeCollection with name {} already exists. Overwriting it.",
                                    name);
        }
    }

    const AttributeCollection* find(std::string_view name) const
    {
        auto it = m_attribute_collections.find(std::string{name});
        if(it != m_attribute_collections.end())
        {
            return it->second.get();
        }
        return nullptr;
    }


    void build_attributes_index_from_attribute_collection(AttributeCollection& ac)
    {
        using AF = AttributeFriend<GeometryAtlas>;

        auto& attr_slots = AF::attribute_slots(ac);

        for(auto&& [attr_name, attr_slot] : attr_slots)
        {
            auto attr = AF::attribute(attr_slot);
            auto it   = m_attr_to_index.find(attr);
            if(it == m_attr_to_index.end())
            {
                IndexT index = static_cast<IndexT>(m_attr_to_index.size());
                m_attr_to_index[attr] = index;
                UIPC_ASSERT(m_index_to_attr.size() == index,
                            "Index to attribute size mismatch, expected {}, got {}",
                            m_index_to_attr.size(),
                            index);
                m_index_to_attr.push_back(attr);
            }
        }
    }

    void build_attributes_index_from_geometry(Geometry& geo)
    {
        using GF = GeometryFriend<GeometryAtlas>;

        vector<std::string>          collection_names;
        vector<AttributeCollection*> attribute_collections;
        GF::attribute_collections(geo, collection_names, attribute_collections);
        for(auto&& ac : attribute_collections)
        {
            build_attributes_index_from_attribute_collection(*ac);
        }
    }

    /**************************************************************
    *                           Serialize
    ***************************************************************/

    Json attributes_to_json()
    {
        AttributeFactory af;
        return af.to_json(m_index_to_attr);
    }

    Json attribute_collection_to_json(const AttributeCollection& ac)
    {
        using AF        = AttributeFriend<GeometryAtlas>;
        auto attr_slots = AF::attribute_slots(ac);

        Json  j    = Json::object();
        auto& meta = j["meta"];
        {
            meta["type"] = UIPC_TO_STRING(AttributeCollection);
        }
        auto& ac_data = j["data"];

        for(auto&& [name, attr_slot] : attr_slots)
        {
            auto attr = AF::attribute(attr_slot);
            auto it   = m_attr_to_index.find(attr);
            if(it != m_attr_to_index.end())
            {
                // attr name -> attr index
                ac_data[name] = it->second;
            }
            else
            {
                UIPC_WARN_WITH_LOCATION("Attribute type {} not registered, so we ignore it",
                                        attr->type_name());
            }
        }
    }

    Json geometry_to_json(Geometry& geo)
    {
        using GF = GeometryFriend<GeometryAtlas>;
        vector<std::string>          collection_names;
        vector<AttributeCollection*> attribute_collections;
        GF::attribute_collections(geo, collection_names, attribute_collections);

        Json  j    = Json::object();
        auto& meta = j["meta"];
        {
            meta["type"] = geo.type();
        }

        auto& data = j["data"];
        for(auto&& [cn, ac] : zip(collection_names, attribute_collections))
        {
            data[cn] = attribute_collection_to_json(*ac);
        }
    }

    Json to_json()
    {
        Json j = Json::object();
        // .meta
        {
            auto& meta = j["meta"];
            {
                meta["type"] = UIPC_TO_STRING(GeometryAtlas);
            }
        }

        //.data
        {
            auto& data = j["data"];
            {
                auto& attribute_collections = data["attribute_collections"];
                // A map of <Name,AttributeCollection>
                for(auto&& [name, ac] : m_attribute_collections)
                {
                    attribute_collections["name"] = attribute_collection_to_json(*ac);
                }

                // An Array of <Geometry>
                auto& geometries = data["geometries"];
                geometries       = Json::array();
                for(auto&& geo : m_geometries)
                {
                    geometries.push_back(geometry_to_json(*geo));
                }

                // An Array of <Attribute>
                data["attributes"] = attributes_to_json();
            }
        }
    }

    /**************************************************************
    *                           Deserialize
    ***************************************************************/

    vector<S<IAttribute>> m_attributes;

    void attributes_from_json(const Json& j)
    {
        AttributeFactory af;
        m_attributes = af.from_json(j);
    }

    S<AttributeCollection> attribute_collection_from_json(const Json& j)
    {
        using AF = AttributeFriend<GeometryAtlas>;
    }

    void from_json(const Json& j)
    {
        UIPC_ASSERT(is_empty(), "Not allow non-empty GeometryAltas generating from json");

        // .meta
        {
            auto it = j.find("meta");
            if(it == j.end())
            {
                UIPC_WARN_WITH_LOCATION("`meta` not found in json, skip.");
                return;
            }

            auto& meta    = *it;
            auto  type_it = meta.find("type");
            if(type_it == meta.end())
            {
                UIPC_WARN_WITH_LOCATION("`meta.type` not found in json, skip.");
                return;
            }

            auto& type = *type_it;
            if(type.get<std::string>() != UIPC_TO_STRING(GeometryAtlas))
            {
                UIPC_WARN_WITH_LOCATION("`meta.type` is not GeometryAtlas, skip.");
                return;
            }
        }

        // .data
        {
            auto it_data = j.find("data");
            if(it_data == j.end())
            {
                UIPC_WARN_WITH_LOCATION("`data` not found in json, skip");
                return;
            }

            auto& data = *it_data;
            {
                auto it_ac = data.find("attribute_collections");
                if(it_ac != data.end())
                {
                    auto& attribute_collections = *it_ac;
                    for(auto&& [name, ac] : attribute_collections.items())
                    {
                        //auto attr_collection = AttributeCollection{};
                        //attr_collection.from_json(ac);
                        //create(name, attr_collection);
                    }
                }
            }

            {
                auto it_geo = data.find("geometries");
                if(it_geo != data.end())
                {
                    auto& geometries = *it_geo;
                    for(auto&& geo : geometries)
                    {
                        //auto geometry = Geometry{};
                        //geometry.from_json(geo);
                        //create(geometry);
                    }
                }
            }

            {
                auto it_attr = data.find("attributes");
                if(it_attr != data.end())
                {
                    attributes_from_json(*it_attr);
                }
            }
        }
    }


    bool is_empty() const
    {
        bool ret = true;
        ret &= m_geometries.empty();
        ret &= m_attribute_collections.empty();
        ret &= m_attr_to_index.empty();
        ret &= m_index_to_attr.empty();
        ret &= m_attributes.empty();
        return ret;
    }
};


GeometryAtlas::GeometryAtlas()
    : m_impl{uipc::make_unique<Impl>()}
{
}

GeometryAtlas::~GeometryAtlas() {}

IndexT GeometryAtlas::create(const Geometry& geo)
{
    return m_impl->create(geo);
}

const Geometry* GeometryAtlas::find(IndexT id) const
{
    return m_impl->find(id);
}

void GeometryAtlas::create(std::string_view name, const AttributeCollection& ac)
{
    m_impl->create(name, ac);
}

const AttributeCollection* GeometryAtlas::find(std::string_view name) const
{
    return m_impl->find(name);
}

Json GeometryAtlas::to_json() const {}
}  // namespace uipc::geometry
