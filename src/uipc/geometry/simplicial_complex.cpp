#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
SimplicialComplex::SimplicialComplex(const AbstractSimplicialComplex& asc,
                                     std::span<const Vector3>         positions)
    : IGeometry{"SimplicialComplex"}
    , m_asc{asc}
{
    UIPC_ASSERT(positions.size() == m_asc.vertices()->size(),
                "Number of vertices in the simplicial complex ({}) does not match the number of positions ({}).",
                m_asc.vertices()->size(),
                positions.size());

    m_vertex_attributes.resize(m_asc.vertices()->size());
    m_edge_attributes.resize(m_asc.edges()->size());
    m_triangle_attributes.resize(m_asc.triangles()->size());
    m_tetrahedron_attributes.resize(m_asc.tetrahedra()->size());

    auto& pos  = m_vertex_attributes.create<Vector3>("position");
    auto  view = pos.view();
    std::copy(positions.begin(), positions.end(), view.begin());
}

AttributeSlot<Vector3>& SimplicialComplex::positions()
{
    return *m_vertex_attributes.find<Vector3>("position");
}

const AttributeSlot<Vector3>& SimplicialComplex::positions() const
{
    return *m_vertex_attributes.find<Vector3>("position");
}
auto SimplicialComplex::vertices() -> VertexAttributes
{
    return VertexAttributes(m_asc.m_vertices, m_vertex_attributes);
}
auto SimplicialComplex::edges() -> EdgeAttributes
{
    return EdgeAttributes(m_asc.m_edges, m_edge_attributes);
}
auto SimplicialComplex::triangles() -> TriangleAttributes
{
    return TriangleAttributes(m_asc.m_triangles, m_triangle_attributes);
}
auto SimplicialComplex::tetrahedra() -> TetrahedronAttributes
{
    return TetrahedronAttributes(m_asc.m_tetrahedra, m_tetrahedron_attributes);
}
}  // namespace uipc::geometry