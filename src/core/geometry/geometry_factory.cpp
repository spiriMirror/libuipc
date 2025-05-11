#include <uipc/geometry/geometry_factory.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/implicit_geometry.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
using Creator = std::function<S<Geometry>(const Json&, span<S<IAttribute>>)>;

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
};


template <std::derived_from<Geometry> T>
static void register_type(std::unordered_map<std::string, Creator>& creators,
                          std::string_view                          type_name)
{
    using GF = GeometryFriend<GeometryFactory>;
    creators.insert(
        {std::string{type_name},
         [](const Json& j, span<S<IAttribute>> attributes) -> S<Geometry>
         {
             S<Geometry> geometry = std::make_shared<T>();

             vector<std::string> names;
             names.reserve(8);
             vector<S<AttributeCollection>> collections;
             collections.reserve(8);

             // get all name-collection pairs
             for(auto& [key, value] : j.items())
             {
                 names.push_back(key);
                 UIPC_ASSERT(false, "NOT IMPL AttributeCollection Fractory YET");
             }

             GF::build_from_attribute_collections(*geometry, names, collections);

             return geometry;
         }});
}


class GeometryFactory::Impl
{
  public:
    static auto& creators()
    {
        static thread_local std::once_flag                           f;
        static thread_local std::unordered_map<std::string, Creator> m_creators;

        std::call_once(f,
                       [&]
                       {
                           // Register all exported geometry types here
                           register_type<SimplicialComplex>(m_creators, builtin::SimplicialComplex);
                           register_type<ImplicitGeometry>(m_creators, builtin::ImplicitGeometry);
                       });


        return m_creators;
    }

    vector<S<Geometry>> from_json(const Json& j, span<S<IAttribute>> attributes)
    {
        return vector<S<Geometry>>();
    }

    Json to_json(span<Geometry*> attributes, unordered_map<IAttribute*, IndexT> attr_to_index)
    {
        return Json();
    }
};


GeometryFactory::GeometryFactory()
    : m_impl{uipc::make_unique<Impl>()}
{
}

GeometryFactory::~GeometryFactory() {}

vector<S<Geometry>> GeometryFactory::from_json(const Json& j, span<S<IAttribute>> attributes)
{
    m_impl->from_json(j, attributes);
}

Json GeometryFactory::to_json(span<Geometry*>                    attributes,
                              unordered_map<IAttribute*, IndexT> attr_to_index)
{
    return m_impl->to_json(attributes, attr_to_index);
}
}  // namespace uipc::geometry
