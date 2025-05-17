#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/common/zip.h>

namespace uipc::geometry
{
SimplicialComplex::SimplicialComplex()
    : Geometry()
{
    UIPC_ASSERT(find("meta") && find("instances"),
                "SimplicialComplex should have meta and instance attributes.");

    // create attribute collections and setup shortcuts
    m_vertex_attributes      = create("vertices");
    m_edge_attributes        = create("edges");
    m_triangle_attributes    = create("triangles");
    m_tetrahedron_attributes = create("tetrahedra");

    // Don't create positions attribute here: some algorithms just **share** the positions attribute.

    // Create a default transform attribute.
    auto      instances = find("instances");  // in base class
    Matrix4x4 I         = Transform::Identity().matrix();
    auto      trans     = instances->create<Matrix4x4>(builtin::transform,
                                              I,  // default identity
                                              false  // don't allow destroy
    );
}

SimplicialComplex::SimplicialComplex(const SimplicialComplex& o)
    : Geometry(o)
{
    // setup shortcuts
    m_vertex_attributes      = find("vertices");
    m_edge_attributes        = find("edges");
    m_triangle_attributes    = find("triangles");
    m_tetrahedron_attributes = find("tetrahedra");
}

AttributeSlot<Matrix4x4>& SimplicialComplex::transforms()
{
    return *instances().template find<Matrix4x4>(builtin::transform);
}

const AttributeSlot<Matrix4x4>& SimplicialComplex::transforms() const
{
    return *instances().template find<Matrix4x4>(builtin::transform);
}

AttributeSlot<Vector3>& SimplicialComplex::positions() noexcept
{
    return *m_vertex_attributes->find<Vector3>(builtin::position);
}

const AttributeSlot<Vector3>& SimplicialComplex::positions() const noexcept
{
    return *m_vertex_attributes->find<Vector3>(builtin::position);
}

auto SimplicialComplex::vertices() noexcept -> VertexAttributes
{
    return VertexAttributes(*m_vertex_attributes);
}

auto SimplicialComplex::vertices() const noexcept -> CVertexAttributes
{
    return CVertexAttributes(*m_vertex_attributes);
}

auto SimplicialComplex::edges() noexcept -> EdgeAttributes
{
    return EdgeAttributes(*m_edge_attributes);
}

auto SimplicialComplex::edges() const noexcept -> CEdgeAttributes
{
    return CEdgeAttributes(*m_edge_attributes);
}

auto SimplicialComplex::triangles() noexcept -> TriangleAttributes
{
    return TriangleAttributes(*m_triangle_attributes);
}

auto SimplicialComplex::triangles() const noexcept -> CTriangleAttributes
{
    return CTriangleAttributes(*m_triangle_attributes);
}

auto SimplicialComplex::tetrahedra() noexcept -> TetrahedronAttributes
{
    return TetrahedronAttributes(*m_tetrahedron_attributes);
}

auto SimplicialComplex::tetrahedra() const noexcept -> CTetrahedronAttributes
{
    return CTetrahedronAttributes(*m_tetrahedron_attributes);
}

S<IGeometry> SimplicialComplex::do_clone() const
{
    return uipc::make_shared<SimplicialComplex>(*this);
}

IndexT SimplicialComplex::dim() const noexcept
{
    if(m_tetrahedron_attributes->size() > 0)
        return 3;
    if(m_triangle_attributes->size() > 0)
        return 2;
    if(m_edge_attributes->size() > 0)
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
