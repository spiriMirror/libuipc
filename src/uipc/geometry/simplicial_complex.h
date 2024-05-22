#pragma once
#include <span>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/simplices.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/abstract_simplicial_complex.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
class SimplicialComplexAttributes
{
  public:
    SimplicialComplexAttributes(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes(SimplicialComplexAttributes&& o)      = default;
    SimplicialComplexAttributes& operator=(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes& operator=(SimplicialComplexAttributes&& o) = default;

    void  resize(size_t size);
    void  reserve(size_t size);
    void  clear();
    SizeT size() const;

    auto topo_view() { return m_topology->view(); }
    auto topo_view() const { return std::as_const(m_topology)->view(); }
    bool topo_is_owned() const;

    template <typename T>
    auto find(std::string_view name)
    {
        return m_attributes.find<T>(name);
    }

  private:
    friend class SimplicialComplex;
    SimplexSlot&         m_topology;
    AttributeCollection& m_attributes;

    SimplicialComplexAttributes(SimplexSlot& topology, AttributeCollection& attributes);
};


class SimplicialComplex : public IGeometry
{
  public:
    using VertexAttributes      = SimplicialComplexAttributes<VertexSlot>;
    using EdgeAttributes        = SimplicialComplexAttributes<EdgeSlot>;
    using TriangleAttributes    = SimplicialComplexAttributes<TriangleSlot>;
    using TetrahedronAttributes = SimplicialComplexAttributes<TetrahedronSlot>;

    SimplicialComplex(const AbstractSimplicialComplex& asc, std::span<const Vector3> positions);

    SimplicialComplex(const SimplicialComplex& o)            = default;
    SimplicialComplex(SimplicialComplex&& o)                 = default;
    SimplicialComplex& operator=(const SimplicialComplex& o) = default;
    SimplicialComplex& operator=(SimplicialComplex&& o)      = default;

    AttributeSlot<Vector3>&       positions();
    const AttributeSlot<Vector3>& positions() const;

    VertexAttributes      vertices();
    EdgeAttributes        edges();
    TriangleAttributes    triangles();
    TetrahedronAttributes tetrahedra();


  private:
    AbstractSimplicialComplex m_asc;
    AttributeCollection       m_vertex_attributes;
    AttributeCollection       m_edge_attributes;
    AttributeCollection       m_triangle_attributes;
    AttributeCollection       m_tetrahedron_attributes;
};
}  // namespace uipc::geometry

#include "details/simplicial_complex.inl"