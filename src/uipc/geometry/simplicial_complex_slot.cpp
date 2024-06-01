#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::geometry
{
GeometrySlot<SimplicialComplex>::GeometrySlot(IndexT id, const SimplicialComplex& simplicial_complex) noexcept
    : m_id(id)
    , m_simplicial_complex(simplicial_complex)
{
}

AttributeSlot<Vector3>& GeometrySlot<SimplicialComplex>::positions() noexcept
{
    return m_simplicial_complex.positions();
}

const AttributeSlot<Vector3>& GeometrySlot<SimplicialComplex>::positions() const noexcept
{
    return m_simplicial_complex.positions();
}

auto GeometrySlot<SimplicialComplex>::vertices() noexcept -> VertexAttributes
{
    return m_simplicial_complex.vertices();
}

auto GeometrySlot<SimplicialComplex>::vertices() const noexcept -> CVertexAttributes
{
    return m_simplicial_complex.vertices();
}

auto GeometrySlot<SimplicialComplex>::edges() noexcept -> EdgeAttributes
{
    return m_simplicial_complex.edges();
}

auto GeometrySlot<SimplicialComplex>::edges() const noexcept -> CEdgeAttributes
{
    return m_simplicial_complex.edges();
}

auto GeometrySlot<SimplicialComplex>::triangles() noexcept -> TriangleAttributes
{
    return m_simplicial_complex.triangles();
}

auto GeometrySlot<SimplicialComplex>::triangles() const noexcept -> CTriangleAttributes
{
    return m_simplicial_complex.triangles();
}

auto GeometrySlot<SimplicialComplex>::tetrahedra() noexcept -> TetrahedronAttributes
{
    return m_simplicial_complex.tetrahedra();
}

auto GeometrySlot<SimplicialComplex>::tetrahedra() const noexcept -> CTetrahedronAttributes
{
    return m_simplicial_complex.tetrahedra();
}

IndexT GeometrySlot<SimplicialComplex>::dim() const noexcept
{
    return m_simplicial_complex.dim();
}

SimplicialComplex& GeometrySlot<SimplicialComplex>::geometry() noexcept
{
    return m_simplicial_complex;
}

const SimplicialComplex& GeometrySlot<SimplicialComplex>::geometry() const noexcept
{
    return m_simplicial_complex;
}

IndexT GeometrySlot<SimplicialComplex>::get_id() const noexcept
{
    return m_id;
}
Geometry& GeometrySlot<SimplicialComplex>::get_geometry() noexcept
{
    return m_simplicial_complex;
}
const Geometry& GeometrySlot<SimplicialComplex>::get_geometry() const noexcept
{
    return m_simplicial_complex;
}
}  // namespace uipc::geometry
