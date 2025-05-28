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
#include <uipc/geometry/shared_attribute_context.h>

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
    static void attribute_collections(Geometry&            geometry,
                                      vector<std::string>& names,
                                      vector<const AttributeCollection*>& collections)
    {
        geometry.collect_attribute_collections(names, collections);
    }
};


static void _build_attributes_index_from_attribute_collection(
    unordered_map<IAttribute*, IndexT>& attr_to_index,
    vector<IAttribute*>&                index_to_attr,
    const AttributeCollection&          ac)
{
    using AF = AttributeFriend<GeometryAtlas>;

    auto& attr_slots = AF::attribute_slots(ac);

    for(auto&& [attr_name, attr_slot] : attr_slots)
    {
        auto attr = AF::attribute(attr_slot);
        auto it   = attr_to_index.find(attr);
        if(it == attr_to_index.end())
        {
            IndexT index        = static_cast<IndexT>(attr_to_index.size());
            attr_to_index[attr] = index;
            UIPC_ASSERT(index_to_attr.size() == index,
                        "Index to attribute size mismatch, expected {}, got {}",
                        index_to_attr.size(),
                        index);
            index_to_attr.push_back(attr);
        }
    }
}


}  // namespace uipc::geometry

namespace uipc::geometry
{
class GeometryAtlas::Impl
{
  public:
    /**************************************************************
    *                          Building
    ***************************************************************/

    vector<S<GeometrySlot>>                            m_geometries;
    unordered_map<std::string, S<AttributeCollection>> m_attribute_collections;

    SerialSharedAttributeContext   m_serial_context;
    DeserialSharedAttributeContext m_deserial_context;

    IndexT create(const Geometry& geometry, bool evolving_only)
    {
        IndexT id = static_cast<IndexT>(m_geometries.size());

        auto slot = gf().create_slot(id, geometry);
        build_attributes_index_from_geometry(slot->geometry());
        slot->id(id);
        m_geometries.push_back(std::move(slot));
        return id;
    }

    S<const GeometrySlot> find(IndexT id) const
    {
        if(id >= m_geometries.size())
        {
            UIPC_WARN_WITH_LOCATION("GeometrySlot with id {} not found", id);
            return nullptr;
        }
        return m_geometries[id];
    }

    void create(std::string_view name, const AttributeCollection& ac, bool evolving_only)
    {
        S<AttributeCollection> new_ac = uipc::make_shared<AttributeCollection>(ac);
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

    S<const AttributeCollection> find(std::string_view name) const
    {
        auto it = m_attribute_collections.find(std::string{name});
        if(it != m_attribute_collections.end())
        {
            return it->second;
        }
        return nullptr;
    }


    void build_attributes_index_from_attribute_collection(const AttributeCollection& ac)
    {
        _build_attributes_index_from_attribute_collection(
            m_serial_context.m_attr_to_index, m_serial_context.m_index_to_attr, ac);
    }

    void build_attributes_index_from_geometry(Geometry& geo)
    {
        using GF = GeometryFriend<GeometryAtlas>;

        vector<std::string>                collection_names;
        vector<const AttributeCollection*> attribute_collections;
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
        return af().to_json(m_serial_context.m_index_to_attr);
    }

    Json attribute_collection_to_json(const AttributeCollection& ac)
    {
        return acf().to_json(ac, m_serial_context);
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

        return gf().to_json(geos_ptr, m_serial_context);
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
        m_deserial_context.m_attribute_slots = af().from_json(j);
    }

    S<AttributeCollection> attribute_collection_from_json(const Json& j)
    {
        return acf().from_json(j, m_deserial_context);
    }

    vector<S<Geometry>> geometries_from_json(const Json& j)
    {
        return gf().from_json(j, m_deserial_context);
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
                            create(name, *ac, false);
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
                        create(*geo, false);
                    }
                }
            }
        }
    }

    void clear()
    {
        m_geometries.clear();
        m_attribute_collections.clear();
        m_serial_context.clear();
        m_deserial_context.clear();
    }
};


GeometryAtlas::GeometryAtlas()
    : m_impl{uipc::make_unique<Impl>()}
{
}

GeometryAtlas::~GeometryAtlas() {}

IndexT GeometryAtlas::create(const Geometry& geo, bool evolving_only)
{
    return m_impl->create(geo, evolving_only);
}

S<const GeometrySlot> GeometryAtlas::find(IndexT id) const
{
    return m_impl->find(id);
}

SizeT GeometryAtlas::geometry_count() const noexcept
{
    return m_impl->m_geometries.size();
}

void GeometryAtlas::create(std::string_view name, const AttributeCollection& ac, bool evolving_only)
{
    m_impl->create(name, ac, evolving_only);
}

S<const AttributeCollection> GeometryAtlas::find(std::string_view name) const
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


namespace uipc::geometry
{
class GeometryAtlasCommit::Impl
{
  public:
    vector<S<GeometryCommit>> m_geometries;
    unordered_map<std::string, S<AttributeCollectionCommit>> m_attribute_collections;

    SerialSharedAttributeContext   m_serial_context;
    DeserialSharedAttributeContext m_deserial_context;

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

    /**************************************************************
    *                          Building
    ***************************************************************/

    IndexT create(const GeometryCommit& geo_commit)
    {
        if(!geo_commit.is_valid())
        {
            UIPC_WARN_WITH_LOCATION("GeometryCommit is not valid, skip.");
            return -1;
        }

        IndexT id = static_cast<IndexT>(m_geometries.size());
        auto   gc = uipc::make_shared<GeometryCommit>(geo_commit);

        if(gc->m_new_geometry)
            build_attributes_index_from_geometry(*gc->m_new_geometry);

        for(auto&& [name, ac_commit] : gc->m_attribute_collections)
        {
            build_attributes_index_from_attribute_collection(ac_commit->attribute_collection());
        }

        m_geometries.push_back(std::move(gc));

        return id;
    }

    S<const GeometryCommit> find(IndexT id) const
    {
        if(id >= m_geometries.size() || id < 0)
        {
            UIPC_WARN_WITH_LOCATION("GeometryCommit with id {} not found", id);
            return nullptr;
        }

        return m_geometries[id];
    }

    void create(std::string_view name, const AttributeCollectionCommit& ac_commit)
    {
        auto it = m_attribute_collections.find(std::string{name});
        auto this_ac_commit = uipc::make_shared<AttributeCollectionCommit>(ac_commit);
        if(it == m_attribute_collections.end())
        {
            m_attribute_collections[std::string{name}] = this_ac_commit;
        }
        else
        {
            it->second = this_ac_commit;
            UIPC_WARN_WITH_LOCATION("AttributeCollection with name {} already exists. Overwriting it.",
                                    name);
        }

        build_attributes_index_from_attribute_collection(
            this_ac_commit->attribute_collection());
    }

    S<const AttributeCollectionCommit> find(std::string_view name) const
    {
        auto it = m_attribute_collections.find(std::string{name});
        if(it != m_attribute_collections.end())
        {
            return it->second;
        }
        return nullptr;
    }

    void build_attributes_index_from_attribute_collection(const AttributeCollection& ac)
    {
        _build_attributes_index_from_attribute_collection(
            m_serial_context.m_attr_to_index, m_serial_context.m_index_to_attr, ac);
    }

    void build_attributes_index_from_geometry(Geometry& geo)
    {
        vector<std::string>                collection_names;
        vector<const AttributeCollection*> attribute_collections;
        GeometryFriend<GeometryAtlas>::attribute_collections(geo, collection_names, attribute_collections);
        for(auto&& ac : attribute_collections)
        {
            build_attributes_index_from_attribute_collection(*ac);
        }
    }
    /**************************************************************
    *                          Serialize
    ***************************************************************/

    Json attributes_to_json()
    {
        return af().to_json(m_serial_context.m_index_to_attr);
    }

    Json attribute_collection_commit_to_json(const AttributeCollectionCommit& ac)
    {
        return acf().commit_to_json(ac, m_serial_context);
    }

    Json geometry_commit_to_json(const GeometryCommit& geo)
    {
        return gf().commit_to_json(geo, m_serial_context);
    }

    Json to_json()
    {
        Json  j    = Json::object();
        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(GeometryAtlasCommit);
        }

        auto& data = j[builtin::__data__];
        {
            // An Array of <Attribute>
            data["attributes"] = attributes_to_json();

            // A Map of <Name,AttributeCollectionCommit>
            auto& attribute_collections = data["attribute_collections"];
            for(auto&& [name, ac] : m_attribute_collections)
            {
                attribute_collections[name] = attribute_collection_commit_to_json(*ac);
            }

            // An Array of <GeometryCommit>
            auto& geometries = data["geometries"];
            for(auto&& geo_commit : m_geometries)
            {
                geometries.push_back(geometry_commit_to_json(*geo_commit));
            }
        }

        return j;
    }

    /**************************************************************
    *                          Deserialize
    ***************************************************************/


    void attributes_from_json(const Json& j)
    {
        m_deserial_context.m_attribute_slots = af().from_json(j);
    }

    S<AttributeCollectionCommit> attribute_collection_commit_from_json(const Json& j)
    {
        return acf().commit_from_json(j, m_deserial_context);
    }

    S<GeometryCommit> geometry_commit_from_json(const Json& j)
    {
        return gf().commit_from_json(j, m_deserial_context);
    }

    void clear()
    {
        m_geometries.clear();
        m_attribute_collections.clear();
        m_serial_context.clear();
        m_deserial_context.clear();
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
            if(type.get<std::string>() != UIPC_TO_STRING(GeometryAtlasCommit))
            {
                UIPC_WARN_WITH_LOCATION("`meta.type` is not GeometryAtlasCommit, skip.");
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
                        auto ac_commit = attribute_collection_commit_from_json(ac_json);
                        if(ac_commit)
                        {
                            create(name, *ac_commit);
                        }
                        else
                        {
                            UIPC_WARN_WITH_LOCATION("AttributeCollectionCommit is null, skip.");
                        }
                    }
                }

                auto it_geo = data.find("geometries");
                if(it_geo != data.end())
                {
                    auto& geometries = *it_geo;
                    for(auto&& geo_commit : geometries)
                    {
                        auto gc = geometry_commit_from_json(geo_commit);
                        if(gc)
                        {
                            create(*gc);
                        }
                        else
                        {
                            UIPC_WARN_WITH_LOCATION("GeometryCommit is null, skip.");
                        }
                    }
                }
            }
        }
    }
};

GeometryAtlasCommit::GeometryAtlasCommit()
    : m_impl{uipc::make_unique<Impl>()}
{
}

GeometryAtlasCommit::~GeometryAtlasCommit() = default;

IndexT GeometryAtlasCommit::create(const GeometryCommit& geo_commit)
{
    return m_impl->create(geo_commit);
}

S<const GeometryCommit> GeometryAtlasCommit::find(IndexT id) const
{
    return m_impl->find(id);
}

void GeometryAtlasCommit::create(std::string_view name, const AttributeCollectionCommit& ac_commit)
{
    return m_impl->create(name, ac_commit);
}

S<const AttributeCollectionCommit> GeometryAtlasCommit::find(std::string_view name) const
{
    return m_impl->find(name);
}

vector<std::string> GeometryAtlasCommit::attribute_collection_names() const noexcept
{
    vector<std::string> names;
    names.reserve(m_impl->m_attribute_collections.size());
    for(auto&& [name, _] : m_impl->m_attribute_collections)
    {
        names.push_back(name);
    }
    return names;
}

SizeT GeometryAtlasCommit::attribute_collection_count() const noexcept
{
    return m_impl->m_attribute_collections.size();
}

SizeT GeometryAtlasCommit::geometry_count() const noexcept
{
    return m_impl->m_geometries.size();
}

Json GeometryAtlasCommit::to_json() const
{
    return m_impl->to_json();
}

void GeometryAtlasCommit::from_json(const Json& j)
{
    m_impl->from_json(j);
}
}  // namespace uipc::geometry
