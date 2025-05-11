#include <uipc/geometry/attribute_factory.h>

namespace uipc::geometry
{
using Creator = std::function<S<IAttribute>(const Json&)>;

template <typename T>
static void register_type(std::unordered_map<std::string, Creator>& creators)
{
    creators.insert({Attribute<T>::type(),  //
                     [](const Json& j) -> S<IAttribute>
                     {
                         auto attribute = std::make_shared<Attribute<T>>();
                         attribute->from_json(j);
                         return attribute;
                     }});
}

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

    vector<S<IAttribute>> from_json(const Json& j)
    {
        vector<S<IAttribute>> attributes;
        UIPC_ASSERT(j.is_array(), "This json must be an array of attributes");
        attributes.reserve(j.size());

        for(SizeT i = 0; i < j.size(); ++i)
        {
            auto& elem    = j[i];
            auto  meta_it = elem.find("meta");
            if(meta_it == elem.end())
            {
                UIPC_WARN_WITH_LOCATION("`meta` info not found, so we ignore it");
                continue;
            }
            auto& meta    = *meta_it;
            auto  type_it = meta.find("type");
            if(type_it == meta.end())
            {
                UIPC_WARN_WITH_LOCATION("`meta.type` info not found, so we ignore it");
                continue;
            }
            auto& type = *type_it;

            auto it = creators().find(type.get<std::string>());
            if(it != creators().end())
            {
                auto& data = elem["data"];
                // call creator
                Creator& creator = it->second;
                auto     attr    = creator(data);
                attributes.push_back(attr);
            }
            else
            {
                UIPC_WARN_WITH_LOCATION("Attribute type {} not registered, so we ignore it",
                                        type.get<std::string>());
            }
        }
        return attributes;
    }

    Json to_json(span<IAttribute*> attributes)
    {
        Json j = Json::array();
        for(auto&& attr : attributes)
        {
            auto it = creators().find(std::string{attr->type_name()});
            if(it != creators().end())
            {
                auto& meta   = j["meta"];
                meta["type"] = attr->type_name();
                auto& data   = j["data"];
                data         = attr->to_json();
            }
            else
            {
                UIPC_WARN_WITH_LOCATION("Attribute type {} not registered, so we ignore it",
                                        attr->type_name());
            }
        }
        return j;
    }
};


AttributeFactory::AttributeFactory()
    : m_impl{uipc::make_unique<Impl>()}
{
}

AttributeFactory::~AttributeFactory() {}

vector<S<IAttribute>> AttributeFactory::from_json(const Json& j)
{
    return m_impl->from_json(j);
}

Json AttributeFactory::to_json(span<IAttribute*> attributes)
{
    return m_impl->to_json(attributes);
}
}  // namespace uipc::geometry
