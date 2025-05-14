#include <uipc/geometry/attribute_collection_factory.h>
#include <uipc/common/macro.h>
#include <uipc/builtin/factory_keyword.h>

namespace uipc::geometry
{
template <>
class AttributeFriend<AttributeCollectionFactory>
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

// A Json representation of the attribute collection looks like this:
// {
//     meta:
//     {
//         type: "AttributeCollection"
//     }
//     data:
//     {
//         name:
//         {
//             index: 0,
//             name: "attr_name",
//             allow_destroy: true
//         }
//     }
//  }

class AttributeCollectionFactory::Impl
{
  public:
    Json attribute_collection_to_json(const AttributeCollection& ac,
                                      unordered_map<IAttribute*, IndexT> attr_to_index)
    {
        using AF        = AttributeFriend<AttributeCollectionFactory>;
        auto attr_slots = AF::attribute_slots(ac);

        Json  j    = Json::object();
        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(AttributeCollection);
        }

        auto& data = j[builtin::__data__];
        {
            for(auto&& [name, attr_slot] : attr_slots)
            {
                auto attr = AF::attribute(attr_slot);
                auto it   = attr_to_index.find(attr);
                if(it != attr_to_index.end())
                {
                    // attr name -> attr index
                    auto& attr_slot_json = data[name];
                    {
                        attr_slot_json["index"] = it->second;
                        attr_slot_json["name"]  = name;
                        attr_slot_json["allow_destroy"] = attr_slot->allow_destroy();
                    }
                }
                else
                {
                    UIPC_WARN_WITH_LOCATION("Attribute at 0x{} not registered, so we ignore it",
                                            (void*)attr);
                }
            }
        }

        return j;
    }

    S<AttributeCollection> attribute_collection_from_json(const Json& j,
                                                          span<S<IAttributeSlot>> attributes)
    {
        using AF = AttributeFriend<AttributeCollectionFactory>;

        // check meta
        {
            auto meta_it = j.find(builtin::__meta__);
            if(meta_it == j.end())
            {
                UIPC_WARN_WITH_LOCATION("`__meta__` not found in json, skip.");
                return {};
            }
            auto& meta = *meta_it;

            auto type_it = meta.find("type");
            if(type_it == meta.end())
            {
                UIPC_WARN_WITH_LOCATION("`__meta__.type` not found in json, skip.");
                return {};
            }
            auto type = type_it->get<std::string>();

            if(type != UIPC_TO_STRING(AttributeCollection))
            {
                UIPC_WARN_WITH_LOCATION("`__meta__.type` is not `AttributeCollection`, skip.");
                return {};
            }
        }

        // build from data
        auto data_it = j.find(builtin::__data__);
        if(data_it == j.end())
        {
            UIPC_WARN_WITH_LOCATION("`__data__` not found in json, skip.");
            return {};
        }

        auto& data = *data_it;
        auto  ac   = uipc::make_shared<AttributeCollection>();
        for(auto&& [name, object] : data.items())
        {
            auto index_it = object.find("index");
            if(index_it == object.end())
            {
                UIPC_WARN_WITH_LOCATION("`index` not found, skip.");
                continue;
            }
            auto index = index_it->get<IndexT>();

            auto name_it = object.find("name");
            if(name_it == object.end())
            {
                UIPC_WARN_WITH_LOCATION("`name` not found, skip.");
                continue;
            }
            auto name = name_it->get<std::string>();

            auto allow_destroy_it = object.find("allow_destroy");
            if(allow_destroy_it == object.end())
            {
                UIPC_WARN_WITH_LOCATION("`allow_destroy` not found, skip.");
                continue;
            }
            auto allow_destroy = allow_destroy_it->get<bool>();

            S<IAttributeSlot> attr = attributes[index];
            ac->resize(attr->size());
            ac->share(name, *attr, allow_destroy);
        }

        return ac;
    }

    Json to_json(const AttributeCollection*         ac,
                 unordered_map<IAttribute*, IndexT> attr_to_index,
                 bool                               evolving_only)
    {
        Json j = Json::object();
        return attribute_collection_to_json(*ac, attr_to_index);
    }

    S<AttributeCollection> from_json(const Json& j, span<S<IAttributeSlot>> attributes)
    {
        UIPC_ASSERT(j.is_object(), "This json must be an object");
        S<AttributeCollection> attribute_collections =
            attribute_collection_from_json(j, attributes);
        return attribute_collections;
    }

    S<AttributeCollection> collect_evolving(const AttributeCollection& source)
    {
        using AF = AttributeFriend<AttributeCollectionFactory>;

        auto ac = uipc::make_shared<AttributeCollection>();
        for(auto&& [name, attr_slot] : AF::attribute_slots(source))
        {
            if(attr_slot->is_evolving())
            {
                ac->share(name, *attr_slot, attr_slot->allow_destroy());
            }
        }
        return ac;
    }

    vector<std::string> collect_removing(const AttributeCollection& current,
                                         const AttributeCollection& reference)
    {
        auto cn = current.names();
        std::ranges::sort(cn);
        auto rn = reference.names();
        std::ranges::sort(rn);
        vector<std::string> names;
        names.reserve(rn.size());

        // compute: ref - cur
        std::ranges::set_difference(rn, cn, std::back_inserter(names));
        return names;
    }
};


AttributeCollectionFactory::AttributeCollectionFactory()
    : m_impl{uipc::make_unique<Impl>()}
{
}

AttributeCollectionFactory::~AttributeCollectionFactory() {}

S<AttributeCollection> AttributeCollectionFactory::collect_evolving(const AttributeCollection& source)
{
    return m_impl->collect_evolving(source);
}

vector<std::string> AttributeCollectionFactory::collect_removing(
    const AttributeCollection& current, const AttributeCollection& reference)
{
    return m_impl->collect_removing(current, reference);
}

S<AttributeCollection> AttributeCollectionFactory::from_json(const Json& j,
                                                             span<S<IAttributeSlot>> attributes)
{
    return m_impl->from_json(j, attributes);
}

Json AttributeCollectionFactory::to_json(const AttributeCollection* acs,
                                         unordered_map<IAttribute*, IndexT> attr_to_index,
                                         bool evolving_only)
{
    return m_impl->to_json(acs, attr_to_index, evolving_only);
}
}  // namespace uipc::geometry