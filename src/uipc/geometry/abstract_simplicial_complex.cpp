#include <uipc/geometry/abstract_simplicial_complex.h>

namespace uipc::geometry
{
AbstractSimplicialComplex::AbstractSimplicialComplex()
    : m_vertices(make_shared<Vertices>())
    , m_edges(make_shared<Edges>())
    , m_triangles(make_shared<Triangles>())
    , m_tetrahedra(make_shared<Tetrahedra>())
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