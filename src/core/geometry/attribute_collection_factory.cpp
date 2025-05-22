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
    Json attribute_collection_to_json(const AttributeCollection&    ac,
                                      SerialSharedAttributeContext& ctx)
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
                auto attr  = AF::attribute(attr_slot);
                auto index = ctx.index_of(attr);
                if(index >= 0)
                {
                    // attr name -> attr index
                    auto& attr_slot_json = data[name];
                    {
                        attr_slot_json["index"] = index;
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
                                                          DeserialSharedAttributeContext& ctx)
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

            // auto name = name_it->get<std::string>();

            auto allow_destroy_it = object.find("allow_destroy");
            if(allow_destroy_it == object.end())
            {
                UIPC_WARN_WITH_LOCATION("`allow_destroy` not found, skip.");
                continue;
            }
            auto allow_destroy = allow_destroy_it->get<bool>();

            S<IAttributeSlot> attr = ctx.attribute_slot_of(index);
            ac->resize(attr->size());
            ac->share(name, *attr, allow_destroy);
        }

        return ac;
    }

    Json to_json(const AttributeCollection& ac, SerialSharedAttributeContext& ctx)
    {
        return attribute_collection_to_json(ac, ctx);
    }

    S<AttributeCollection> from_json(const Json& j, DeserialSharedAttributeContext& ctx)
    {
        UIPC_ASSERT(j.is_object(), "This json must be an object");
        S<AttributeCollection> attribute_collections =
            attribute_collection_from_json(j, ctx);
        return attribute_collections;
    }

    void diff(const AttributeCollection& current,
              const AttributeCollection& reference,
              AttributeCollection&       diff_copy,
              vector<std::string>&       removed_names)
    {
        using AF = AttributeFriend<AttributeCollectionFactory>;

        for(auto&& [name, attr_slot] : AF::attribute_slots(current))
        {
            // check if the attribute is evolving
            // if so, we need to copy it to the diff_copy
            if(attr_slot->is_evolving())
            {
                diff_copy.share(name, *attr_slot, attr_slot->allow_destroy());
            }

            // check if the attribute is newer than the reference
            // if so we also need to copy it to the diff_copy
            auto ret_attribute = reference.find(name);
            if(ret_attribute)
            {
                if(attr_slot->last_modified() > ret_attribute->last_modified())
                {
                    diff_copy.share(name, *attr_slot, attr_slot->allow_destroy());
                }
            }
        }

        // collect the removed names
        removed_names.clear();
        auto cn = current.names();
        std::ranges::sort(cn);
        auto rn = reference.names();
        std::ranges::sort(rn);
        removed_names.reserve(rn.size());

        // compute: ref - cur
        // e.g.
        // ref = {"A", "B", "C"}
        // cur = {"A", "D"}
        // removed_names = {"B", "C"}
        std::ranges::set_difference(rn, cn, std::back_inserter(removed_names));
    }

    Json commit_to_json(const AttributeCollectionCommit& acs, SerialSharedAttributeContext& ctx)
    {
        Json  j    = Json::object();
        auto& meta = j[builtin::__meta__];
        {
            meta["type"] = UIPC_TO_STRING(AttributeCollectionCommit);
        }

        auto& data = j[builtin::__data__];
        {
            data["attribute_collection"] =
                attribute_collection_to_json(acs.m_attribute_collection, ctx);
            data["removed_names"] = acs.m_removed_names;
        }

        return j;
    }

    S<AttributeCollectionCommit> commit_from_json(const Json& j,
                                                  DeserialSharedAttributeContext& ctx)
    {
        using AF = AttributeFriend<AttributeCollectionFactory>;
        auto acc = uipc::make_shared<AttributeCollectionCommit>();
        // check meta
        do
        {
            {
                auto meta_it = j.find(builtin::__meta__);
                if(meta_it == j.end())
                {
                    UIPC_WARN_WITH_LOCATION("`__meta__` not found in json, skip.");
                    break;
                }
                auto& meta = *meta_it;

                auto type_it = meta.find("type");
                if(type_it == meta.end())
                {
                    UIPC_WARN_WITH_LOCATION("`__meta__.type` not found in json, skip.");
                    break;
                }
                auto type = type_it->get<std::string>();

                if(type != UIPC_TO_STRING(AttributeCollectionCommit))
                {
                    UIPC_WARN_WITH_LOCATION("`__meta__.type` is not `AttributeCollectionCommit`, skip.");
                    break;
                }
            }

            // build from data
            auto data_it = j.find(builtin::__data__);
            if(data_it == j.end())
            {
                UIPC_WARN_WITH_LOCATION("`__data__` not found in json, skip.");
                break;
            }

            auto& data = *data_it;

            auto attribute_collection_it = data.find("attribute_collection");
            if(attribute_collection_it == data.end())
            {
                UIPC_WARN_WITH_LOCATION("`attribute_collection` not found, skip.");
                break;
            }

            auto& attribute_collection = *attribute_collection_it;
            auto ac = attribute_collection_from_json(attribute_collection, ctx);
            if(ac)
            {
                acc->m_attribute_collection = std::move(*ac);
            }

            auto removed_names_it = data.find("removed_names");

            if(removed_names_it == data.end())
            {
                UIPC_WARN_WITH_LOCATION("`removed_names` not found, skip.");
                break;
            }

            auto& removed_names  = *removed_names_it;
            acc->m_removed_names = removed_names.get<vector<std::string>>();
        } while(0);  // for safe break;

        return acc;
    }
};


AttributeCollectionFactory::AttributeCollectionFactory()
    : m_impl{uipc::make_unique<Impl>()}
{
}

AttributeCollectionFactory::~AttributeCollectionFactory() = default;

S<AttributeCollection> AttributeCollectionFactory::from_json(const Json& j,
                                                             DeserialSharedAttributeContext& ctx)
{
    return m_impl->from_json(j, ctx);
}

Json AttributeCollectionFactory::to_json(const AttributeCollection&    acs,
                                         SerialSharedAttributeContext& ctx)
{
    return m_impl->to_json(acs, ctx);
}

Json AttributeCollectionFactory::commit_to_json(const AttributeCollectionCommit& acs,
                                                SerialSharedAttributeContext& ctx)
{
    return m_impl->commit_to_json(acs, ctx);
}

S<AttributeCollectionCommit> AttributeCollectionFactory::commit_from_json(
    const Json& j, DeserialSharedAttributeContext& ctx)
{
    return m_impl->commit_from_json(j, ctx);
}

AttributeCollectionCommit AttributeCollectionFactory::diff(const AttributeCollection& current,
                                                           const AttributeCollection& reference)
{
    AttributeCollectionCommit commit;
    m_impl->diff(current, reference, commit.m_attribute_collection, commit.m_removed_names);
    return commit;
}
}  // namespace uipc::geometry