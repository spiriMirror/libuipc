#include <uipc/common/format.h>
#include <uipc/common/readable_type_name.h>

namespace uipc::geometry
{
template <typename T>
AttributeSlot<T>::AttributeSlot(std::string_view m_name, S<Attribute<T>> attribute, bool allow_destroy)
    : m_name(m_name)
    , m_attribute(std::move(attribute))
    , m_allow_destroy(allow_destroy)
    , m_last_modified(std::chrono::high_resolution_clock::now())
{
}

template <typename U>
[[nodiscard]] span<U> view(AttributeSlot<U>& slot)
{
    UIPC_ASSERT(&slot, "You are trying to access a nullptr attribute slot, please check if the attribute name is correct");
    slot.make_owned();
    // record the modification time
    slot.m_last_modified = std::chrono::high_resolution_clock::now();
    return view(*slot.m_attribute);
}

template <typename T>
[[nodiscard]] span<const T> AttributeSlot<T>::view() const noexcept
{
    UIPC_ASSERT(this, "You are trying to access a nullptr attribute slot, please check if the attribute name is correct");
    return m_attribute->view();
}

template <typename T>
std::string_view AttributeSlot<T>::get_name() const noexcept
{
    return m_name;
}

template <typename T>
bool AttributeSlot<T>::get_allow_destroy() const noexcept
{
    return m_allow_destroy;
}

template <typename T>
bool AttributeSlot<T>::get_is_evolving() const noexcept
{
    return m_is_evolving;
}

template <typename T>
void AttributeSlot<T>::set_is_evolving(bool v) noexcept
{
    m_is_evolving = v;
}

template <typename T>
void AttributeSlot<T>::do_make_owned()
{
    // if I am not the only one who is using the attribute, then I need to make a copy_from
    if(m_attribute.use_count() > 1)
        m_attribute = uipc::make_shared<Attribute<T>>(*m_attribute);
}

template <typename T>
S<IAttributeSlot> AttributeSlot<T>::do_clone(std::string_view name, bool allow_destroy) const
{
    return uipc::make_shared<AttributeSlot<T>>(
        name, std::static_pointer_cast<Attribute<T>>(m_attribute), allow_destroy);
}

template <typename T>
S<IAttributeSlot> AttributeSlot<T>::do_clone_empty(std::string_view name, bool allow_destroy) const
{
    return uipc::make_shared<AttributeSlot<T>>(
        name, uipc::make_shared<Attribute<T>>(m_attribute->m_default_value), allow_destroy);
}

template <typename T>
TimePoint AttributeSlot<T>::get_last_modified() const noexcept
{
    return m_last_modified;
}

template <typename T>
IAttribute& AttributeSlot<T>::get_attribute() noexcept
{
    return *m_attribute;
}

template <typename T>
const IAttribute& AttributeSlot<T>::get_attribute() const noexcept
{
    return *m_attribute;
}

template <typename T>
SizeT uipc::geometry::AttributeSlot<T>::get_use_count() const noexcept
{
    return m_attribute.use_count();
}
}  // namespace uipc::geometry