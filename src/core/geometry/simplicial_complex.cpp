#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>

namespace uipc::geometry
{
SimplicialComplex::SimplicialComplex()
{
    // Don't create positions attribute here: some algorithms just **share** the positions attribute.

    // Create a default transform attribute.
    Matrix4x4 I = Transform::Identity().matrix();
    auto trans  = m_intances.create<Matrix4x4, false>(builtin::transform, I);
}

AttributeSlot<Matrix4x4>& SimplicialComplex::transforms()
{
    return *m_intances.template find<Matrix4x4>(builtin::transform);
}

const AttributeSlot<Matrix4x4>& SimplicialComplex::transforms() const
{
    return *m_intances.template find<Matrix4x4>(builtin::transform);
}

AttributeSlot<Vector3>& SimplicialComplex::positions() noexcept
{
    return *m_vertex_attributes.find<Vector3>(builtin::position);
}

const AttributeSlot<Vector3>& SimplicialComplex::positions() const noexcept
{
    return *m_vertex_attributes.find<Vector3>(builtin::position);
}

auto SimplicialComplex::vertices() noexcept -> VertexAttributes
{
    return VertexAttributes(m_vertex_attributes);
}

auto SimplicialComplex::vertices() const noexcept -> CVertexAttributes
{
    return CVertexAttributes(m_vertex_attributes);
}

auto SimplicialComplex::edges() noexcept -> EdgeAttributes
{
    return EdgeAttributes(m_edge_attributes);
}

auto SimplicialComplex::edges() const noexcept -> CEdgeAttributes
{
    return CEdgeAttributes(m_edge_attributes);
}

auto SimplicialComplex::triangles() noexcept -> TriangleAttributes
{
    return TriangleAttributes(m_triangle_attributes);
}

auto SimplicialComplex::triangles() const noexcept -> CTriangleAttributes
{
    return CTriangleAttributes(m_triangle_attributes);
}

auto SimplicialComplex::tetrahedra() noexcept -> TetrahedronAttributes
{
    return TetrahedronAttributes(m_tetrahedron_attributes);
}

auto SimplicialComplex::tetrahedra() const noexcept -> CTetrahedronAttributes
{
    return CTetrahedronAttributes(m_tetrahedron_attributes);
}

Json SimplicialComplex::do_to_json() const
{
    auto base          = Geometry::do_to_json();
    base["vertices"]   = vertices().to_json();
    base["edges"]      = edges().to_json();
    base["triangles"]  = triangles().to_json();
    base["tetrahedra"] = tetrahedra().to_json();
    return base;
}

void SimplicialComplex::do_collect_attribute_collections(vector<std::string>& names,
                                                         vector<AttributeCollection*>& collections)
{
    Geometry::do_collect_attribute_collections(names, collections);

    names.push_back("vertices");
    collections.push_back(&m_vertex_attributes);

    names.push_back("edges");
    collections.push_back(&m_edge_attributes);

    names.push_back("triangles");
    collections.push_back(&m_triangle_attributes);

    names.push_back("tetrahedra");
    collections.push_back(&m_tetrahedron_attributes);
}

IndexT SimplicialComplex::dim() const noexcept
{
    if(m_tetrahedron_attributes.size() > 0)
        return 3;
    if(m_triangle_attributes.size() > 0)
        return 2;
    if(m_edge_attributes.size() > 0)
        return 1;
    return 0;
}

std::string_view SimplicialComplex::get_type() const noexcept
{
    return builtin::SimplicialComplex;
}
}  // namespace uipc::geometry

namespace fmt
{
appender fmt::formatter<uipc::geometry::SimplicialComplex>::format(
    const uipc::geometry::SimplicialComplex& c, format_context& ctx) const
{
    return fmt::format_to(ctx.out(),
                          R"({}
vertices({}):{};
edges({}):{};
triangles({}):{}; 
tetrahedra({}):{};)",
                          static_cast<const uipc::geometry::Geometry&>(c),
                          c.vertices().size(),
                          c.vertices(),
                          c.edges().size(),
                          c.edges(),
                          c.triangles().size(),
                          c.triangles(),
                          c.tetrahedra().size(),
                          c.tetrahedra());
}
}  // namespace fmt
