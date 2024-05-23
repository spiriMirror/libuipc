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
/**
 * @brief A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.
 */
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

    /**
     * @brief Get a non-const view of the topology.
     * 
     * @warning This function may cause a data clone if the topology is shared.
     */
    auto topo_view() { return m_topology->view(); }
    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    auto topo_view() const { return std::as_const(m_topology)->view(); }
    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    bool topo_is_shared() const;

    /**
     * @brief Find an attribute by type and name, if the attribute does not exist, return nullptr.
     */
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

/**
 * @brief A simplicial complex is a collection of simplices, for example, vertices, edges, triangles, and tetrahedra.
 * 
 * $K = (V, E, F, T)$, where $V$ is the set of vertices, $E$ is the set of edges,
 * $F$ is the set of triangles, and $T$ is the set of tetrahedra.
 * 
 * A simple example:
 * ```cpp
 *  auto mesh = geometry::tetmesh(Vs, Ts);
 *  auto VA  = mesh.vertices();
 *  auto TA  = mesh.tetrahedra();
 *  auto pos = VA.find<Vector3>("position");
 *  auto const_view = std::as_const(pos)->view();
 *  auto non_const_view = pos->view();
 * ```
 * 
 * @warning An non-const view of the attribute may cause data clone if the attribute is shared.
 * To avoid data clone, use `std::as_const(this_instance).attribute_view()` instead.
 * 
 */
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

    /**
     * @brief A short cut to get the positions of the vertices
     */
    AttributeSlot<Vector3>& positions();
    /**
     * @brief A short cut to get the positions of the vertices
     */
    const AttributeSlot<Vector3>& positions() const;

    /**
     * @return A wrapper of the vertices and its attributes of the simplicial complex.
     */
    VertexAttributes vertices();
    /**
     * @return A wrapper of the edges and its attributes of the simplicial complex.
     * 
     * @sa EdgeAttributes
     */
    EdgeAttributes edges();
    /**
     * @return A wrapper of the triangles and its attributes of the simplicial complex.
     * 
     */
    TriangleAttributes triangles();
    /**
    * @return A wrapper of the tetrahedra and its attributes of the simplicial complex.
    */
    TetrahedronAttributes tetrahedra();

    /**
    * @brief Get the dimension of the simplicial complex.
    */
    IndexT dim() const;

  private:
    AbstractSimplicialComplex m_asc;
    AttributeCollection       m_vertex_attributes;
    AttributeCollection       m_edge_attributes;
    AttributeCollection       m_triangle_attributes;
    AttributeCollection       m_tetrahedron_attributes;
};
}  // namespace uipc::geometry

#include "details/simplicial_complex.inl"