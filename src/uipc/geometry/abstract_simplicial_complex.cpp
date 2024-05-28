#include <uipc/geometry/abstract_simplicial_complex.h>

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

U<ISimplexSlot> ISimplexSlot::clone() const
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

span<IndexT> view(VertexSlot& slot)
{
    slot.do_make_owned();
    return view(*slot.m_simplices);
}

U<VertexSlot> VertexSlot::clone() const
{
    return std::make_unique<VertexSlot>(m_simplices);
}

SizeT VertexSlot::get_use_count() const noexcept
{
    return m_simplices.use_count();
}

U<ISimplexSlot> VertexSlot::do_clone() const
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

AbstractSimplicialComplex::AbstractSimplicialComplex()
    : m_vertices(std::make_shared<Vertices>())
    , m_edges(std::make_shared<Edges>())
    , m_triangles(std::make_shared<Triangles>())
    , m_tetrahedra(std::make_shared<Tetrahedra>())
{
}

AbstractSimplicialComplex::AbstractSimplicialComplex(const AbstractSimplicialComplex& o)
    : m_vertices(o.m_vertices.m_simplices)
    , m_edges(o.m_edges.m_simplices)
    , m_triangles(o.m_triangles.m_simplices)
    , m_tetrahedra(o.m_tetrahedra.m_simplices)
{
}

AbstractSimplicialComplex& AbstractSimplicialComplex::operator=(const AbstractSimplicialComplex& o)
{
    if(std::addressof(o) != this)
    {
        m_vertices   = VertexSlot(m_vertices.m_simplices);
        m_edges      = EdgeSlot(o.m_edges.m_simplices);
        m_triangles  = TriangleSlot(o.m_triangles.m_simplices);
        m_tetrahedra = TetrahedronSlot(o.m_tetrahedra.m_simplices);
    }
    return *this;
}

AbstractSimplicialComplex::AbstractSimplicialComplex(AbstractSimplicialComplex&& o) noexcept
    : m_vertices(std::move(o.m_vertices))
    , m_edges(std::move(o.m_edges))
    , m_triangles(std::move(o.m_triangles))
    , m_tetrahedra(std::move(o.m_tetrahedra))
{
}

AbstractSimplicialComplex& AbstractSimplicialComplex::operator=(AbstractSimplicialComplex&& o) noexcept
{
    if(std::addressof(o) != this)
    {
        m_vertices   = std::move(o.m_vertices);
        m_edges      = std::move(o.m_edges);
        m_triangles  = std::move(o.m_triangles);
        m_tetrahedra = std::move(o.m_tetrahedra);
    }
    return *this;
}

VertexSlot& AbstractSimplicialComplex::vertices() noexcept
{
    return m_vertices;
}

const VertexSlot& AbstractSimplicialComplex::vertices() const noexcept
{
    return m_vertices;
}

EdgeSlot& AbstractSimplicialComplex::edges() noexcept
{
    return m_edges;
}

const EdgeSlot& AbstractSimplicialComplex::edges() const noexcept
{
    return m_edges;
}

TriangleSlot& AbstractSimplicialComplex::triangles() noexcept
{
    return m_triangles;
}

const TriangleSlot& AbstractSimplicialComplex::triangles() const noexcept
{
    return m_triangles;
}

TetrahedronSlot& AbstractSimplicialComplex::tetrahedra() noexcept
{
    return m_tetrahedra;
}

const TetrahedronSlot& AbstractSimplicialComplex::tetrahedra() const noexcept
{
    return m_tetrahedra;
}
}  // namespace uipc::geometry