#include <uipc/geometry/attribute_factory.h>
#include <uipc/geometry/attribute_slot.h>
#include <uipc/builtin/factory_keyword.h>

namespace uipc::geometry
{
// Must Return AttributeSlot, we need to use the clone facility of IAttributeSlot
using Creator = std::function<S<IAttributeSlot>(const Json&)>;

template <typename T>
static void register_type(std::unordered_map<std::string, Creator>& creators)
{
    creators.insert({Attribute<T>::type(),  //
                     [](const Json& j) -> S<IAttributeSlot>
                     {
                         auto attribute = uipc::make_shared<Attribute<T>>();
                         attribute->from_json(j);

                         // an AttributeSlot without ownership
                         // don't need name (never used)
                         // allow_destroy is false (no care)
                         auto attr_slot =
                             uipc::make_shared<AttributeSlot<T>>("", attribute, false);

                         return attr_slot;
                     }});
}


// A Json representation of the attribute looks like this:
// {
//     __meta__:
//     {
//         base: "IAttribute",
//         type: "T"
//     }
//     __data__:
//     {
//          // json representation of the attribute
//     }
//  }

class AttributeFactory::Impl
{
  public:
    static auto& creators()
    {
        static thread_local std::once_flag                           f;
        static thread_local std::unordered_map<std::string, Creator> m_creators;

        std::call_once(f,
                       [&] {
        // Register all exported attribute types here
#define UIPC_ATTRIBUTE_EXPORT_DEF(T)                                           \
    ::uipc::geometry::register_type<T>(m_creators);

#include <uipc/geometry/details/attribute_export_types.inl>

#undef UIPC_ATTRIBUTE_EXPORT_DEF
                       });
        return m_creators;
    }

    Json to_json(span<IAttribute*> attributes)
    {
        Json j = Json::array();
        for(auto&& attr : attributes)
        {
            auto it = creators().find(std::string{attr->type_name()});

            if(it != creators().end())
            {
                Json  elem   = Json::object();
                auto& meta   = elem[builtin::__meta__];
                meta["base"] = "IAttribute";
                meta["type"] = attr->type_name();
                auto& data   = elem[builtin::__data__];
                data         = attr->to_json();
                j.push_back(elem);
            }
            else
            {
                UIPC_WARN_WITH_LOCATION("Attribute<{}> not registered, so we ignore it",
                                        attr->type_name());
            }
        }
        return j;
    }

    vector<S<IAttributeSlot>> from_json(const Json& j)
    {
        vector<S<IAttributeSlot>> attributes;
        UIPC_ASSERT(j.is_array(), "This json must be an array of attributes");
        attributes.reserve(j.size());

        for(SizeT i = 0; i < j.size(); ++i)
        {
            auto& items   = j[i];
            auto  meta_it = items.find(builtin::__meta__);
            if(meta_it == items.end())
            {
                UIPC_WARN_WITH_LOCATION("`__meta__` info not found, so we ignore it");
                continue;
            }
            auto& meta    = *meta_it;
            auto  base_it = meta.find("base");
            if(base_it == meta.end())
            {
                UIPC_WARN_WITH_LOCATION("`base` info not found, so we ignore it");
                continue;
            }
            auto& base = *base_it;
            if(base.get<std::string>() != "IAttribute")
            {
                UIPC_WARN_WITH_LOCATION("`base` info not match, so we ignore it");
                continue;
            }
            auto type_it = meta.find("type");
            if(type_it == meta.end())
            {
                UIPC_WARN_WITH_LOCATION("`type` info not found, so we ignore it");
                continue;
            }
            auto type = type_it->get<std::string>();

            auto& data       = items[builtin::__data__];
            auto  creator_it = creators().find(type);
            if(creator_it != creators().end())
            {
                // call creator
                Creator& creator = creator_it->second;
                auto     attr    = creator(data);
                attributes.push_back(attr);
            }
            else
            {
                UIPC_WARN_WITH_LOCATION("Attribute<{}> not registered, so we ignore it", type);
            }
        }
        return attributes;
    }
};


AttributeFactory::AttributeFactory()
    : m_impl{uipc::make_unique<Impl>()}
{
}

AttributeFactory::~AttributeFactory() {}

vector<S<IAttributeSlot>> AttributeFactory::from_json(const Json& j)
{
    return m_impl->from_json(j);
}

Json AttributeFactory::to_json(span<IAttribute*> attributes)
{
    return m_impl->to_json(attributes);
}
}  // namespace uipc::geometry
