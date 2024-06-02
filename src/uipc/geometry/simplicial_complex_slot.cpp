#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::geometry
{
GeometrySlotT<SimplicialComplex>::GeometrySlotT(IndexT id, const SimplicialComplex& simplicial_complex) noexcept
    : GeometrySlot{id}
    , m_simplicial_complex{simplicial_complex}
{
}

AttributeSlot<Vector3>& GeometrySlotT<SimplicialComplex>::positions() noexcept
{
    return m_simplicial_complex.positions();
}

const AttributeSlot<Vector3>& GeometrySlotT<SimplicialComplex>::positions() const noexcept
{
    return m_simplicial_complex.positions();
}

auto GeometrySlotT<SimplicialComplex>::vertices() noexcept -> VertexAttributes
{
    return m_simplicial_complex.vertices();
}

auto GeometrySlotT<SimplicialComplex>::vertices() const noexcept -> CVertexAttributes
{
    return m_simplicial_complex.vertices();
}

auto GeometrySlotT<SimplicialComplex>::edges() noexcept -> EdgeAttributes
{
    return m_simplicial_complex.edges();
}

auto GeometrySlotT<SimplicialComplex>::edges() const noexcept -> CEdgeAttributes
{
    return m_simplicial_complex.edges();
}

auto GeometrySlotT<SimplicialComplex>::triangles() noexcept -> TriangleAttributes
{
    return m_simplicial_complex.triangles();
}

auto GeometrySlotT<SimplicialComplex>::triangles() const noexcept -> CTriangleAttributes
{
    return m_simplicial_complex.triangles();
}

auto GeometrySlotT<SimplicialComplex>::tetrahedra() noexcept -> TetrahedronAttributes
{
    return m_simplicial_complex.tetrahedra();
}

auto GeometrySlotT<SimplicialComplex>::tetrahedra() const noexcept -> CTetrahedronAttributes
{
    return m_simplicial_complex.tetrahedra();
}

IndexT GeometrySlotT<SimplicialComplex>::dim() const noexcept
{
    return m_simplicial_complex.dim();
}

SimplicialComplex& GeometrySlotT<SimplicialComplex>::geometry() noexcept
{
    return m_simplicial_complex;
}

const SimplicialComplex& GeometrySlotT<SimplicialComplex>::geometry() const noexcept
{
    return m_simplicial_complex;
}

Geometry& GeometrySlotT<SimplicialComplex>::get_geometry() noexcept
{
    return m_simplicial_complex;
}

const Geometry& GeometrySlotT<SimplicialComplex>::get_geometry() const noexcept
{
    return m_simplicial_complex;
}
}  // namespace uipc::geometry
