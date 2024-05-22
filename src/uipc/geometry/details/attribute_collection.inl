#include "attribute_collection.h"
namespace uipc::geometry
{
template <typename T>
AttributeSlot<T>::AttributeSlot(std::string_view m_name, S<Attribute<T>> attribute)
    : IAttributeSlot(m_name)
    , m_attribute(std::move(attribute))
{
}

template <typename T>
AttributeSlot<T>& AttributeCollection::create(std::string_view name)
{
    auto n  = std::string{name};
    auto it = m_attributes.find(n);
    if(it != m_attributes.end())
    {
        throw AttributeAlreadyExist{name};
    }
    auto S = std::make_shared<Attribute<T>>();
    S->resize(m_size);
    auto U          = std::make_unique<AttributeSlot<T>>(name, S);
    auto P          = U.get();
    m_attributes[n] = std::move(U);
    return *P;
}

template <typename T>
AttributeSlot<T>& AttributeCollection::share(std::string_view        name,
                                             const AttributeSlot<T>& slot)
{
    return static_cast<AttributeSlot<T>&>(
        this->share(name, static_cast<const IAttributeSlot&>(slot)));
}

template <typename T>
AttributeSlot<T>* AttributeCollection::find(std::string_view name)
{
    return dynamic_cast<AttributeSlot<T>*>(this->find(name));
}

template <typename T>
const AttributeSlot<T>* AttributeCollection::find(std::string_view name) const
{
    return dynamic_cast<const AttributeSlot<T>*>(this->find(name));
}

template <typename T>
std::span<T> AttributeSlot<T>::view()
{
    make_owned();
    return m_attribute->view();
}

template <typename T>
std::span<const T> AttributeSlot<T>::view() const
{
    return m_attribute->view();
}

template <typename T>
void AttributeSlot<T>::do_make_owned()
{
    // if I am not the only one who is using the attribute, then I need to make a copy
    if(m_attribute.use_count() > 1)
        m_attribute = std::make_shared<Attribute<T>>(*m_attribute);
}

template <typename T>
U<IAttributeSlot> AttributeSlot<T>::do_clone() const
{
    return std::make_unique<AttributeSlot<T>>(
        name(), std::static_pointer_cast<Attribute<T>>(m_attribute));
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
}  // namespace uipc::geometry
