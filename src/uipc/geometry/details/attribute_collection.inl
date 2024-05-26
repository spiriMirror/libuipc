#include <format>
#include "attribute_collection.h"

namespace uipc::geometry
{
template <typename T>
AttributeSlot<T>::AttributeSlot(std::string_view m_name, S<Attribute<T>> attribute, bool allow_destroy)
    : m_name(m_name)
    , m_attribute(std::move(attribute))
    , m_allow_destroy(allow_destroy)
{
}

template <typename T, bool AllowDestroy>
P<AttributeSlot<T>> AttributeCollection::create(std::string_view name, const T& default_value)
{
    auto n  = std::string{name};
    auto it = m_attributes.find(n);
    if(it != m_attributes.end())
    {
        throw AttributeAlreadyExist{
            std::format("Attribute with name [{}] already exist!", name)};
    }
    auto S = std::make_shared<Attribute<T>>(default_value);
    S->resize(m_size);
    auto U          = std::make_shared<AttributeSlot<T>>(name, S, AllowDestroy);
    m_attributes[n] = U;
    return U;
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

template <typename T>
span<const T> AttributeSlot<T>::view() const
{
    return m_attribute->view();
}

template <typename T>
std::string_view AttributeSlot<T>::get_name() const
{
    return m_name;
}

template <typename T>
bool AttributeSlot<T>::get_allow_destroy() const
{
    return m_allow_destroy;
}

template <typename T>
void AttributeSlot<T>::do_make_owned()
{
    // if I am not the only one who is using the attribute, then I need to make a copy
    if(m_attribute.use_count() > 1)
        m_attribute = std::make_shared<Attribute<T>>(*m_attribute);
}

template <typename T>
S<IAttributeSlot> AttributeSlot<T>::do_clone() const
{
    return std::make_shared<AttributeSlot<T>>(
        name(), std::static_pointer_cast<Attribute<T>>(m_attribute), m_allow_destroy);
}

template <typename T>
IAttribute& AttributeSlot<T>::get_attribute()
{
    return *m_attribute;
}

template <typename T>
const IAttribute& AttributeSlot<T>::get_attribute() const
{
    return *m_attribute;
}
template <typename T>
SizeT uipc::geometry::AttributeSlot<T>::get_use_count() const
{
    return m_attribute.use_count();
}

template <typename U>
[[nodiscard]] span<U> view(AttributeSlot<U>& slot)
{
    slot.make_owned();
    return view(*slot.m_attribute);
}
}  // namespace uipc::geometry
