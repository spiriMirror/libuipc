#include "simplices.h"
namespace uipc::geometry
{
template <typename Derived>
    requires std::is_base_of_v<ISimplices, Derived>
Derived& ISimplices::cast()
{
    return dynamic_cast<Derived&>(*this);
}

template <typename Derived>
    requires std::is_base_of_v<ISimplices, Derived>
const Derived& ISimplices::cast() const
{
    return dynamic_cast<const Derived&>(*this);
}

template <IndexT N>
IndexT Simplices<N>::dim() const
{
    return N;
}

template <IndexT N>
std::span<const Vector<IndexT, N + 1>> Simplices<N>::view() const
{
    return m_simplices;
}

template <IndexT N>
std::span<Vector<IndexT, N + 1>> Simplices<N>::view()
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
S<ISimplices> Simplices<N>::do_clone() const
{
    return std::make_shared<Simplices<N>>(*this);
}
}  // namespace uipc::geometry
