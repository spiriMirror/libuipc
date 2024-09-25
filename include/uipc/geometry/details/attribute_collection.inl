#include <uipc/common/format.h>

namespace uipc::geometry
{
#define UIPC_ATTRIBUTE_EXPORT_DEF(T)                                           \
    extern template S<AttributeSlot<T>> AttributeCollection::create<T, true>(  \
        std::string_view, const T&);                                           \
    extern template S<AttributeSlot<T>> AttributeCollection::create<T, false>( \
        std::string_view, const T&)

#include "attribute_export_types.inl"

#undef UIPC_ATTRIBUTE_EXPORT_DEF

template <typename T>
S<AttributeSlot<T>> AttributeCollection::share(std::string_view        name,
                                               const AttributeSlot<T>& slot)
{
    return std::static_pointer_cast<AttributeSlot<T>>(
        this->share(name, static_cast<const IAttributeSlot&>(slot)));
}

template <typename T>
S<AttributeSlot<T>> AttributeCollection::find(std::string_view name)
{
    auto slot = this->find(name);
    return std::dynamic_pointer_cast<AttributeSlot<T>>(slot);
}

template <typename T>
S<const AttributeSlot<T>> AttributeCollection::find(std::string_view name) const
{
    auto slot = this->find(name);
    return std::dynamic_pointer_cast<const AttributeSlot<T>>(slot);
}
}  // namespace uipc::geometry
