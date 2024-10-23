#include <uipc/common/format.h>

namespace uipc::geometry
{
template <typename T>
S<AttributeSlot<T>> AttributeCollection::share(std::string_view        name,
                                               const AttributeSlot<T>& slot,
                                               bool allow_destroy)
{
    return std::static_pointer_cast<AttributeSlot<T>>(
        this->share(name, static_cast<const IAttributeSlot&>(slot), allow_destroy));
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
