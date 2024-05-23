namespace uipc::geometry
{
template <IndexT N>
IndexT Simplices<N>::get_dim() const
{
    return N;
}

template <IndexT N>
span<const Vector<IndexT, N + 1>> Simplices<N>::view() const
{
    return m_simplices;
}

template <IndexT N>
span<Vector<IndexT, N + 1>> Simplices<N>::view()
{
    return m_simplices;
}

template <IndexT N>
SizeT Simplices<N>::get_size() const
{
    return m_simplices.size();
}

template <IndexT N>
void Simplices<N>::do_resize(SizeT N)
{
    m_simplices.resize(N);
}

template <IndexT N>
void Simplices<N>::do_clear()
{
    m_simplices.clear();
}

template <IndexT N>
void Simplices<N>::do_reserve(SizeT N)
{
    m_simplices.reserve(N);
}

template <IndexT N>
S<ITopoElements> Simplices<N>::do_clone() const
{
    return std::make_shared<Simplices<N>>(*this);
}
}  // namespace uipc::geometry
