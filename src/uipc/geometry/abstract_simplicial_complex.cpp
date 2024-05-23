#include <uipc/geometry/abstract_simplicial_complex.h>

namespace uipc::geometry
{
void ISimplexSlot::make_owned()
{
    if(!is_shared())
        return;
    do_make_owned();
}

SizeT ISimplexSlot::use_count() const
{
    return get_use_count();
}

U<ISimplexSlot> ISimplexSlot::clone() const
{
    return do_clone();
}

ISimplices& ISimplexSlot::simplices()
{
    return get_simplices();
}

const ISimplices& ISimplexSlot::simplices() const
{
    return get_simplices();
}

ISimplices* ISimplexSlot::operator->()
{
    make_owned();
    return &simplices();
}

const ISimplices* ISimplexSlot::operator->() const
{
    return &simplices();
}

VertexSlot::VertexSlot(S<Vertices> vertices)
    : m_simplices(vertices)
{
}

Vertices* VertexSlot::operator->()
{
    return static_cast<Vertices*>(ISimplexSlot::operator->());
}

const Vertices* VertexSlot::operator->() const
{
    return static_cast<const Vertices*>(ISimplexSlot::operator->());
}

U<VertexSlot> VertexSlot::clone() const
{
    return std::make_unique<VertexSlot>(m_simplices);
}

SizeT VertexSlot::get_use_count() const
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

ISimplices& VertexSlot::get_simplices()
{
    return *m_simplices;
}

const ISimplices& VertexSlot::get_simplices() const
{
    return *m_simplices;
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

VertexSlot& AbstractSimplicialComplex::vertices()
{
    return m_vertices;
}

const VertexSlot& AbstractSimplicialComplex::vertices() const
{
    return m_vertices;
}

EdgeSlot& AbstractSimplicialComplex::edges()
{
    return m_edges;
}

const EdgeSlot& AbstractSimplicialComplex::edges() const
{
    return m_edges;
}

TriangleSlot& AbstractSimplicialComplex::triangles()
{
    return m_triangles;
}

const TriangleSlot& AbstractSimplicialComplex::triangles() const
{
    return m_triangles;
}

TetrahedronSlot& AbstractSimplicialComplex::tetrahedra()
{
    return m_tetrahedra;
}

const TetrahedronSlot& AbstractSimplicialComplex::tetrahedra() const
{
    return m_tetrahedra;
}

bool ISimplexSlot::is_shared() const
{
    return use_count() != 1;
}
SizeT ISimplexSlot::size() const
{
    return simplices().size();
}
}  // namespace uipc::geometry