#include <uipc/common/json_eigen.h>
namespace uipc::geometry
{
template <IndexT N>
backend::BufferView Simplices<N>::get_backend_view() const noexcept
{
    return m_backend_view;
}
template <IndexT N>
IndexT Simplices<N>::get_dim() const noexcept
{
    return N;
}

template <IndexT N>
span<const Vector<IndexT, N + 1>> Simplices<N>::view() const noexcept
{
    return m_simplices;
}

template <IndexT N>
span<Vector<IndexT, N + 1>> Simplices<N>::view() noexcept
{
    return m_simplices;
}

template <IndexT N>
SizeT Simplices<N>::get_size() const noexcept
{
    return m_simplices.size();
}

template <IndexT N>
void Simplices<N>::do_resize(SizeT n)
{
    m_simplices.resize(n);
}

template <IndexT N>
void Simplices<N>::do_clear()
{
    m_simplices.clear();
}

template <IndexT N>
void Simplices<N>::do_reserve(SizeT n)
{
    m_simplices.reserve(n);
}

template <IndexT N>
void Simplices<N>::do_reorder(span<const SizeT> O)
{
    auto old_simplices = m_simplices;
    for(SizeT i = 0; i < O.size(); ++i)
    {
        m_simplices[i] = old_simplices[O[i]];
    }
}

template <IndexT N>
Json Simplices<N>::do_to_json(SizeT i) const
{
    Json j;
    j = m_simplices[i];
    return j;
}

template <IndexT N>
S<ITopoElements> Simplices<N>::do_clone() const
{
    return uipc::make_shared<Simplices<N>>(*this);
}
}  // namespace uipc::geometry
