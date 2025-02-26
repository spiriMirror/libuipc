#include <uipc/geometry/abstract_simplicial_complex.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>

namespace uipc::geometry
{
auto AbstractSimplicialComplex::vertices() noexcept -> VertexAttributes
{
    return VertexAttributes(m_vertex_attributes);
}

auto AbstractSimplicialComplex::vertices() const noexcept -> CVertexAttributes
{
    return CVertexAttributes(m_vertex_attributes);
}

auto AbstractSimplicialComplex::edges() noexcept -> EdgeAttributes
{
    return EdgeAttributes(m_edge_attributes);
}

auto AbstractSimplicialComplex::edges() const noexcept -> CEdgeAttributes
{
    return CEdgeAttributes(m_edge_attributes);
}

auto AbstractSimplicialComplex::triangles() noexcept -> TriangleAttributes
{
    return TriangleAttributes(m_triangle_attributes);
}

auto AbstractSimplicialComplex::triangles() const noexcept -> CTriangleAttributes
{
    return CTriangleAttributes(m_triangle_attributes);
}

auto AbstractSimplicialComplex::tetrahedra() noexcept -> TetrahedronAttributes
{
    return TetrahedronAttributes(m_tetrahedron_attributes);
}

auto AbstractSimplicialComplex::tetrahedra() const noexcept -> CTetrahedronAttributes
{
    return CTetrahedronAttributes(m_tetrahedron_attributes);
}

Json AbstractSimplicialComplex::do_to_json() const
{
    auto base          = Geometry::do_to_json();
    base["vertices"]   = vertices().to_json();
    base["edges"]      = edges().to_json();
    base["triangles"]  = triangles().to_json();
    base["tetrahedra"] = tetrahedra().to_json();
    return base;
}

IndexT AbstractSimplicialComplex::dim() const noexcept
{
    if(m_tetrahedron_attributes.size() > 0)
        return 3;
    if(m_triangle_attributes.size() > 0)
        return 2;
    if(m_edge_attributes.size() > 0)
        return 1;
    return 0;
}

std::string_view AbstractSimplicialComplex::get_type() const noexcept
{
    return builtin::AbstractSimplicialComplex;
}
}  // namespace uipc::geometry

namespace fmt
{
appender fmt::formatter<uipc::geometry::AbstractSimplicialComplex>::format(
    const uipc::geometry::AbstractSimplicialComplex& c, format_context& ctx) const
{
    return fmt::format_to(ctx.out(),
                          R"({}
vertices({}):{};
edges({}):{};
triangles({}):{}; 
tetrahedra({}):{};)",
                          static_cast<const uipc::geometry::Geometry&>(c),
                          c.vertices().size(),
                          c.m_vertex_attributes,
                          c.edges().size(),
                          c.m_edge_attributes,
                          c.triangles().size(),
                          c.m_triangle_attributes,
                          c.tetrahedra().size(),
                          c.m_tetrahedron_attributes);
}
}  // namespace fmt
