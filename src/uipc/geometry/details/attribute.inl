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
span<const T> Attribute<T>::view() const
{
    return m_values;
}

template <typename T>
SizeT Attribute<T>::get_size() const
{
    return m_values.size();
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
}  // namespace uipc::geometry
