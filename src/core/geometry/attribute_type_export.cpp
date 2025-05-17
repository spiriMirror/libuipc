#include <uipc/geometry/attribute_collection.h>
namespace uipc::geometry
{
// NOTE:
// To make the allocation of the attribute always in the uipc_core.dll/.so's memory space,
// we need to explicitly instantiate the template function in the .cpp file.
template <typename T>
S<AttributeSlot<T>> AttributeCollection::create(std::string_view name,
                                                const T&         default_value,
                                                bool             allow_destory)
{
    auto n  = string{name};
    auto it = m_attributes.find(n);
    if(it != m_attributes.end())
    {
        throw AttributeCollectionError{
            fmt::format("Attribute with name [{}] already exist!", name)};
    }
    auto A = uipc::make_shared<Attribute<T>>(default_value);
    A->resize(m_size);
    auto S = uipc::make_shared<AttributeSlot<T>>(name, A, allow_destory);
    m_attributes[n] = S;
    return S;
}

#define UIPC_ATTRIBUTE_EXPORT_DEF(T)                                           \
    template UIPC_CORE_API S<AttributeSlot<T>> AttributeCollection::create<T>( \
        std::string_view, const T&, bool);

#include <uipc/geometry/details/attribute_export_types.inl>

#undef UIPC_ATTRIBUTE_EXPORT_DEF
}  // namespace uipc::geometry