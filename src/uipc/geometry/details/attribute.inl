namespace uipc::geometry
{
template <typename T>
span<T> Attribute<T>::view()
{
    return m_values;
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
    m_values.resize(N);
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
