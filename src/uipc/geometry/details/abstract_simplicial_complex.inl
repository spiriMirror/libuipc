#include "abstract_simplicial_complex.h"
namespace uipc::geometry
{
template <IndexT N>
SimplexSlot<N>::SimplexSlot(S<Simplices<N>> simplices)
    : m_simplices(std::move(simplices))
{
}
template <IndexT N>
Simplices<N>* SimplexSlot<N>::operator->()
{
    return static_cast<Simplices<N>*>(ISimplexSlot::operator->());
}
template <IndexT N>
const Simplices<N>* SimplexSlot<N>::operator->() const
{
    return static_cast<const Simplices<N>*>(ISimplexSlot::operator->());
}
template <IndexT N>
U<SimplexSlot<N>> SimplexSlot<N>::clone() const
{
    return std::make_unique<SimplexSlot<N>>(m_simplices);
}
template <IndexT N>
SizeT SimplexSlot<N>::get_use_count() const
{
    return m_simplices.use_count();
}
template <IndexT N>
U<ISimplexSlot> SimplexSlot<N>::do_clone() const
{
    return clone();
}
template <IndexT N>
void SimplexSlot<N>::do_make_owned()
{
    m_simplices = std::make_shared<Simplices<N>>(*m_simplices);
}
template <IndexT N>
ISimplices& SimplexSlot<N>::get_simplices()
{
    return *m_simplices;
}
template <IndexT N>
const ISimplices& SimplexSlot<N>::get_simplices() const
{
    return *m_simplices;
}
}  // namespace uipc::geometry
