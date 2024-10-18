#include <uipc/diff_sim/parameter_bundle.h>
#include <uipc/common/set.h>

namespace uipc::diff_sim
{
class ParameterBundle::Interface
{
  public:
    virtual ~Interface() = default;
};

using FactoryFunction =
    std::function<U<ParameterBundle::Interface>(S<geometry::GeometrySlot>,    //
                                                S<geometry::IAttributeSlot>,  //
                                                S<geometry::AttributeSlot<IndexT>>  //
                                                )>;


template <typename T>
class ParameterBundleT : public ParameterBundle::Interface
{
  public:
    ParameterBundleT(S<geometry::GeometrySlot>          geo_slot,
                     S<geometry::AttributeSlot<T>>      parm_attr,
                     S<geometry::AttributeSlot<IndexT>> mapping)
        : m_geo_slot(geo_slot)
        , m_parm_attr(parm_attr)
        , m_mapping(mapping)
    {
        UIPC_ASSERT(m_parm_attr->size() == m_mapping->size(),
                    "ParameterBundleT: size of {}({}) and {}({}) must be the same",
                    m_parm_attr->name(),
                    m_parm_attr->size(),
                    m_mapping->name(),
                    m_mapping->size());

        auto map = m_mapping->view();
    }

  private:
    S<geometry::GeometrySlot>          m_geo_slot;
    S<geometry::AttributeSlot<T>>      m_parm_attr;
    S<geometry::AttributeSlot<IndexT>> m_mapping;
};

class ParameterBundleFactory
{
  public:
    template <typename T>
    static void register_type(unordered_map<string, FactoryFunction>& factory_functions)
    {
        factory_functions[readable_type_name<T>()] =
            [](S<geometry::GeometrySlot>          geo_slot,
               S<geometry::IAttributeSlot>        parm_attr,
               S<geometry::AttributeSlot<IndexT>> mapping)
        {
            return uipc::static_pointer_cast<ParameterBundle::Interface>(
                uipc::make_unique<ParameterBundleT<Float>>(
                    geo_slot,
                    std::static_pointer_cast<geometry::AttributeSlot<Float>>(parm_attr),
                    mapping));
        };
    }

    static U<ParameterBundle::Interface> create(S<geometry::GeometrySlot> geo_slot,
                                                S<geometry::IAttributeSlot> parm_attr,
                                                S<geometry::AttributeSlot<IndexT>> mapping)
    {
        std::once_flag                                flag;
        static unordered_map<string, FactoryFunction> factory_functions;
        std::call_once(flag,
                       [&]
                       {
                           register_type<Float>(factory_functions);
                           register_type<Vector2>(factory_functions);
                           register_type<Vector3>(factory_functions);
                       });

        auto it = factory_functions.find(std::string{parm_attr->type_name()});
        if(it == factory_functions.end())
            throw uipc::Exception(std::format("Unknown support attribute type: {} for ParameterBundle",
                                              parm_attr->type_name()));
    }
};


ParameterBundle::ParameterBundle(S<geometry::GeometrySlot>          geo_slot,
                                 S<geometry::IAttributeSlot>        parm_attr,
                                 S<geometry::AttributeSlot<IndexT>> mapping)
{
    m_impl = ParameterBundleFactory::create(geo_slot, parm_attr, mapping);
}

ParameterBundle::~ParameterBundle() {}
}  // namespace uipc::diff_sim
