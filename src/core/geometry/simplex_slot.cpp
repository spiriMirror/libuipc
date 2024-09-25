#include <uipc/geometry/simplex_slot.h>
#include <uipc/common/range.h>

namespace uipc::geometry
{
bool ISimplexSlot::is_shared() const
{
    return use_count() != 1;
}
SizeT ISimplexSlot::size() const
{
    return simplices().size();
}

void ISimplexSlot::share(const ISimplexSlot& other)
{
    if(std::addressof(other) == this)
        return;
    do_share(other);
}

void ISimplexSlot::reorder(span<const SizeT> O)
{
    make_owned();
    do_reorder(O);
}

void ISimplexSlot::resize(SizeT size)
{
    make_owned();
    do_resize(size);
}

void ISimplexSlot::reserve(SizeT capacity)
{
    do_reserve(capacity);
}

void ISimplexSlot::clear()
{
    make_owned();
    do_clear();
}

Json ISimplexSlot::to_json(SizeT i) const noexcept
{
    return simplices().to_json(i);
}

Json ISimplexSlot::to_json() const noexcept
{
    Json j;
    j["name"]   = "topo";
    j["values"] = simplices().to_json();
    return j;
}

void ISimplexSlot::make_owned()
{
    if(!is_shared())
        return;
    do_make_owned();
}

SizeT ISimplexSlot::use_count() const noexcept
{
    return get_use_count();
}

S<ISimplexSlot> ISimplexSlot::clone() const
{
    return do_clone();
}

ISimplices& ISimplexSlot::simplices() noexcept
{
    return get_simplices();
}

const ISimplices& ISimplexSlot::simplices() const noexcept
{
    return get_simplices();
}

VertexSlot::VertexSlot(S<Vertices> vertices) noexcept
    : m_simplices(vertices)
{
}

span<const IndexT> VertexSlot::view() const
{
    return std::as_const(*m_simplices).view();
}

backend::BufferView backend_view(const ISimplexSlot& s) noexcept
{
    return backend_view(s.simplices());
}

UIPC_CORE_API span<IndexT> view(VertexSlot& slot)
{
    slot.do_make_owned();
    return view(*slot.m_simplices);
}

S<VertexSlot> VertexSlot::clone() const
{
    return uipc::make_shared<VertexSlot>(m_simplices);
}

SizeT VertexSlot::get_use_count() const noexcept
{
    return m_simplices.use_count();
}

S<ISimplexSlot> VertexSlot::do_clone() const
{
    return clone();
}

void VertexSlot::do_make_owned()
{
    m_simplices = uipc::make_shared<Vertices>(*m_simplices);
}

ISimplices& VertexSlot::get_simplices() noexcept
{
    return *m_simplices;
}

const ISimplices& VertexSlot::get_simplices() const noexcept
{
    return *m_simplices;
}

void VertexSlot::do_reorder(span<const SizeT> O) noexcept
{
    m_simplices->reorder(O);
}

void VertexSlot::do_resize(SizeT size)
{
    m_simplices->resize(size);
}

void VertexSlot::do_reserve(SizeT capacity)
{
    m_simplices->reserve(capacity);
}

void VertexSlot::do_clear()
{
    m_simplices->clear();
}

void VertexSlot::do_share(const ISimplexSlot& other)
{
    auto& o     = static_cast<const VertexSlot&>(other);
    m_simplices = o.m_simplices;
}
}  // namespace uipc::geometry


// NOTE: To make all allocations in the uipc_core.dll/.so's memory space,
// we need to explicitly instantiate the template class in the .cpp file.
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

template class UIPC_CORE_API SimplexSlot<1>;
template class UIPC_CORE_API SimplexSlot<2>;
template class UIPC_CORE_API SimplexSlot<3>;

template UIPC_CORE_API span<SimplexSlot<1>::ValueT> view<1>(SimplexSlot<1>& slot);
template UIPC_CORE_API span<SimplexSlot<2>::ValueT> view<2>(SimplexSlot<2>& slot);
template UIPC_CORE_API span<SimplexSlot<3>::ValueT> view<3>(SimplexSlot<3>& slot);
}  // namespace uipc::geometry
