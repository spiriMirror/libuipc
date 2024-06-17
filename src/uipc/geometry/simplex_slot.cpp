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
    return std::make_shared<VertexSlot>(m_simplices);
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
    m_simplices = std::make_shared<Vertices>(*m_simplices);
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
