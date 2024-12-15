#include <uipc/common/fmt_eigen.h>

namespace uipc::geometry
{
template <IndexT N>
SimplexSlot<N>::SimplexSlot(S<Simplices<N>> simplices) noexcept
    : m_simplices(std::move(simplices))
{
}

template <IndexT N>
auto SimplexSlot<N>::view() const noexcept -> span<const ValueT>
{
    return std::as_const(*m_simplices).view();
}

template <IndexT N>
auto view(SimplexSlot<N>& slot) -> span<typename SimplexSlot<N>::ValueT>
{
    slot.do_make_owned();
    return view(*slot.m_simplices);
}

template <IndexT N>
S<SimplexSlot<N>> SimplexSlot<N>::clone() const
{
    return uipc::make_shared<SimplexSlot<N>>(m_simplices);
}
template <IndexT N>
SizeT SimplexSlot<N>::get_use_count() const noexcept
{
    return m_simplices.use_count();
}

template <IndexT N>
S<ISimplexSlot> SimplexSlot<N>::do_clone() const
{
    return clone();
}

template <IndexT N>
void SimplexSlot<N>::do_make_owned()
{
    m_simplices = uipc::make_shared<Simplices<N>>(*m_simplices);
}

template <IndexT N>
ISimplices& SimplexSlot<N>::get_simplices() noexcept
{
    return *m_simplices;
}

template <IndexT N>
const ISimplices& SimplexSlot<N>::get_simplices() const noexcept
{
    return *m_simplices;
}

template <IndexT N>
void SimplexSlot<N>::do_reorder(span<const SizeT> O) noexcept
{
    m_simplices->reorder(O);
}

template <IndexT N>
void SimplexSlot<N>::do_resize(SizeT size)
{
    do_make_owned();
    m_simplices->resize(size);
}

template <IndexT N>
void SimplexSlot<N>::do_reserve(SizeT capacity)
{
    do_make_owned();
    m_simplices->reserve(capacity);
}

template <IndexT N>
void SimplexSlot<N>::do_clear()
{
    do_make_owned();
    m_simplices->clear();
}
template <IndexT N>
void SimplexSlot<N>::do_share(const ISimplexSlot& other)
{
    const auto& other_slot = static_cast<const SimplexSlot<N>&>(other);
    m_simplices            = other_slot.m_simplices;
}
}  // namespace uipc::geometry
