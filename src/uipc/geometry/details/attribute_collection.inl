#include <uipc/common/format.h>

namespace uipc::geometry
{
template <typename T, bool AllowDestroy>
P<AttributeSlot<T>> AttributeCollection::create(std::string_view name, const T& default_value)
{
    auto n  = string{name};
    auto it = m_attributes.find(n);
    if(it != m_attributes.end())
    {
        throw GeometryAttributeError{
            fmt::format("Attribute with name [{}] already exist!", name)};
    }
    auto A = uipc::make_shared<Attribute<T>>(default_value);
    A->resize(m_size);
    auto S = uipc::make_shared<AttributeSlot<T>>(name, A, AllowDestroy);
    m_attributes[n] = S;
    return S;
}

template <typename T>
P<AttributeSlot<T>> AttributeCollection::share(std::string_view        name,
                                               const AttributeSlot<T>& slot)
{
    return std::static_pointer_cast<AttributeSlot<T>>(
        this->share(name, static_cast<const IAttributeSlot&>(slot)).lock());
}


template <typename T>
P<AttributeSlot<T>> AttributeCollection::find(std::string_view name)
{
    auto slot = this->find(name);
    return std::dynamic_pointer_cast<AttributeSlot<T>>(slot.lock());
}

template <typename T>
P<const AttributeSlot<T>> AttributeCollection::find(std::string_view name) const
{
    auto slot = this->find(name);
    return std::dynamic_pointer_cast<const AttributeSlot<T>>(slot.lock());
}
}  // namespace uipc::geometry
