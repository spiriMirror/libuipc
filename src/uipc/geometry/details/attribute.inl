#include "attribute.h"
namespace uipc::geometry
{
template <typename T>
Attribute<T>::Attribute(const T& default_value) noexcept
    : m_values{}
    , m_default_value{default_value}
{
}

template <typename T>
span<const T> Attribute<T>::view() const noexcept
{
    return m_values;
}

template <typename T>
SizeT Attribute<T>::get_size() const
{
    return m_values.size();
}

template <typename T>
backend::BufferView Attribute<T>::get_backend_view() const noexcept
{
    return m_backend_view;
}

template <typename T>
void Attribute<T>::do_resize(SizeT N)
{
    m_values.resize(N, this->m_default_value);
}

template <typename T>
void Attribute<T>::do_clear()
{
    m_values.clear();
}

template <typename T>
void Attribute<T>::do_reserve(SizeT N)
{
    m_values.reserve(N);
}

template <typename T>
S<IAttribute> Attribute<T>::do_clone() const
{
    return std::make_shared<Attribute<T>>(*this);
}
template <typename T>
inline S<IAttribute> Attribute<T>::do_clone_empty() const
{
    return std::make_shared<Attribute<T>>(m_default_value);
}
template <typename T>
void uipc::geometry::Attribute<T>::do_reorder(span<const SizeT> O) noexcept
{
    auto old_values = m_values;
    for(SizeT i = 0; i < O.size(); ++i)
        m_values[i] = old_values[O[i]];
}
template <typename T>
void uipc::geometry::Attribute<T>::do_copy_from(span<const SizeT> O,
                                                const IAttribute& other) noexcept
{
    auto& other_attr = static_cast<const Attribute<T>&>(other);
    for(SizeT i = 0; i < O.size(); ++i)
        m_values[i] = other_attr.m_values[O[i]];
}
}  // namespace uipc::geometry
