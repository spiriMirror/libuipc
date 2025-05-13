#include <uipc/geometry/geometry_atlas.h>
#include <unordered_set>
#include <unordered_map>
#include <uipc/common/macro.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/span.h>
#include <uipc/geometry/attribute_factory.h>
#include <uipc/geometry/attribute_collection_factory.h>
#include <uipc/geometry/geometry_factory.h>
#include <uipc/common/zip.h>
#include <uipc/builtin/factory_keyword.h>

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

    vector<S<GeometrySlot>>                            m_geometries;
    unordered_map<std::string, S<AttributeCollection>> m_attribute_collections;
    unordered_map<IAttribute*, IndexT>                 m_attr_to_index;
    vector<IAttribute*>                                m_index_to_attr;

    IndexT create(const Geometry& geometry)
    {
        IndexT id = static_cast<IndexT>(m_geometries.size());

        auto slot = gf().create_slot(id, geometry);
        build_attributes_index_from_geometry(slot->geometry());
        slot->id(id);
        m_geometries.push_back(std::move(slot));
        return id;
    }

    const GeometrySlot* find(IndexT id) const
    {
        if(id >= m_geometries.size())
        {
            UIPC_WARN_WITH_LOCATION("GeometrySlot with id {} not found", id);
            return nullptr;
        }
        return m_geometries[id].get();
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

    Json attributes_to_json() { return af().to_json(m_index_to_attr); }

    Json attribute_collection_to_json(const AttributeCollection& ac)
    {
        return acf().to_json(&ac, m_attr_to_index);
    }

    Json geometries_to_json(span<S<GeometrySlot>> geos)
    {


        vector<Geometry*> geos_ptr;
        geos_ptr.reserve(geos.size());
        std::transform(geos.begin(),
                       geos.end(),
                       std::back_inserter(geos_ptr),
                       [](const S<GeometrySlot>& geo)
                       { return &geo->geometry(); });

        return gf().to_json(geos_ptr, m_attr_to_index);
    }

    Json to_json()
    {
        Json  j    = Json::object();
        auto& meta = j[builtin::__meta__];
        {

            meta["type"] = UIPC_TO_STRING(GeometryAtlas);
        }

        auto& data = j[builtin::__data__];
        {
            // An Array of <Attribute>
            data["attributes"] = attributes_to_json();

            // A Map of <Name,AttributeCollection>
            auto& attribute_collections = data["attribute_collections"];
            for(auto&& [name, ac] : m_attribute_collections)
            {
                attribute_collections[name] = attribute_collection_to_json(*ac);
            }

            // An Array of <Geometry>
            auto& geometries = data["geometries"];
            geometries       = geometries_to_json(m_geometries);
        }

        return j;
    }

    /**************************************************************
    *                           Deserialize
    ***************************************************************/

    vector<S<IAttributeSlot>> m_attributes;

    static AttributeFactory& af()
    {
        static thread_local AttributeFactory af;
        return af;
    }

    static AttributeCollectionFactory& acf()
    {
        static thread_local AttributeCollectionFactory acf;
        return acf;
    }

    static GeometryFactory& gf()
    {
        static thread_local GeometryFactory gf;
        return gf;
    }

    void attributes_from_json(const Json& j)
    {
        m_attributes = af().from_json(j);
    }

    S<AttributeCollection> attribute_collection_from_json(const Json& j)
    {
        return acf().from_json(j, m_attributes);
    }

    vector<S<Geometry>> geometries_from_json(const Json& j)
    {
        return gf().from_json(j, m_attributes);
    }

    void from_json(const Json& j)
    {
        clear();

        // __meta__
        {
            auto it = j.find(builtin::__meta__);
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

        // __data__
        {
            auto it_data = j.find(builtin::__data__);
            if(it_data == j.end())
            {
                UIPC_WARN_WITH_LOCATION("`data` not found in json, skip");
                return;
            }

            auto& data = *it_data;
            {

                auto it_attr = data.find("attributes");
                if(it_attr != data.end())
                {
                    attributes_from_json(*it_attr);
                }

                auto it_ac = data.find("attribute_collections");
                if(it_ac != data.end())
                {
                    auto& attribute_collections = *it_ac;
                    for(auto&& [name, ac_json] : attribute_collections.items())
                    {
                        auto ac = attribute_collection_from_json(ac_json);
                        if(ac)
                        {
                            create(name, *ac);
                        }
                    }
                }

                auto it_geo = data.find("geometries");
                if(it_geo != data.end())
                {
                    auto& geometries = *it_geo;
                    auto  geos       = geometries_from_json(geometries);
                    for(auto&& geo : geos)
                    {
                        create(*geo);
                    }
                }
            }
        }
    }

    void clear()
    {
        m_geometries.clear();
        m_attribute_collections.clear();
        m_attr_to_index.clear();
        m_index_to_attr.clear();
        m_attributes.clear();
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

const GeometrySlot* GeometryAtlas::find(IndexT id) const
{
    return m_impl->find(id);
}

SizeT GeometryAtlas::geometry_count() const noexcept
{
    return m_impl->m_geometries.size();
}

void GeometryAtlas::create(std::string_view name, const AttributeCollection& ac)
{
    m_impl->create(name, ac);
}

const AttributeCollection* GeometryAtlas::find(std::string_view name) const
{
    return m_impl->find(name);
}

SizeT GeometryAtlas::attribute_collection_count() const noexcept
{
    return m_impl->m_attribute_collections.size();
}

vector<std::string> GeometryAtlas::attribute_collection_names() const noexcept
{
    vector<std::string> names;
    names.reserve(m_impl->m_attribute_collections.size());
    for(auto&& [name, _] : m_impl->m_attribute_collections)
    {
        names.push_back(name);
    }
    return names;
}

Json GeometryAtlas::to_json() const
{
    return m_impl->to_json();
}

void GeometryAtlas::from_json(const Json& j)
{
    m_impl->from_json(j);
}
}  // namespace uipc::geometry
