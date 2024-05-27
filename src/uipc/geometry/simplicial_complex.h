#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/abstract_simplicial_complex.h>
#include <uipc/geometry/simplicial_complex_attributes.h>

namespace uipc::geometry
{
/**
 * @brief A simplicial complex is a collection of simplices.
 * 
 * In $\mathbb{R}^3$, a simplicial complex is defined as follows:
 * $$
 * K = (V, E, F, T),
 * $$
 * where $V$ is the set of vertices, $E$ is the set of edges, $F$ is the set of triangles, and $T$ is the set of tetrahedra.
 * 
 * @sa [Tutorial/Geometry](../../../../tutorial/geometry.md)
 */
class SimplicialComplex : public Geometry
{
  public:
    /**
     * @brief Alias for the vertex attributes
     * 
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using VertexAttributes = SimplicialComplexAttributes<VertexSlot>;
    /**
     * @brief Alias for the edge attributes
     * 
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using EdgeAttributes = SimplicialComplexAttributes<EdgeSlot>;
    /**
     * @brief Alias for the triangle attributes
     * 
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using TriangleAttributes = SimplicialComplexAttributes<TriangleSlot>;
    /**
     * @brief Alias for the tetrahedron attributes
     *
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using TetrahedronAttributes = SimplicialComplexAttributes<TetrahedronSlot>;

    SimplicialComplex(const AbstractSimplicialComplex& asc, span<const Vector3> positions);
    SimplicialComplex(const SimplicialComplex& o)            = default;
    SimplicialComplex(SimplicialComplex&& o)                 = default;
    SimplicialComplex& operator=(const SimplicialComplex& o) = default;
    SimplicialComplex& operator=(SimplicialComplex&& o)      = default;

    /**
     * @brief Get the positions of the vertices
     * 
     * @return AttributeSlot<Vector3>& 
     */
    [[nodiscard]] AttributeSlot<Vector3>& positions();

    /**
     * @brief A short cut to get the positions of the vertices
     * 
     * @return const AttributeSlot<Vector3>& 
     */
    [[nodiscard]] const AttributeSlot<Vector3>& positions() const;

    /**
     * @brief A wrapper of the vertices and its attributes of the simplicial complex.
     * 
     * @return VertexAttributes 
     */
    [[nodiscard]] VertexAttributes vertices();

    /**
     * @brief A wrapper of the edges and its attributes of the simplicial complex.
     * 
     * @return EdgeAttributes 
     */
    [[nodiscard]] EdgeAttributes edges();
    /**
     * @brief  A wrapper of the triangles and its attributes of the simplicial complex.
     * 
     * @return TriangleAttributes 
     */
    [[nodiscard]] TriangleAttributes triangles();
    /**
     * @brief A wrapper of the tetrahedra and its attributes of the simplicial complex.
     * 
     * @return TetrahedronAttributes 
     */
    [[nodiscard]] TetrahedronAttributes tetrahedra();

    /**
     * @brief Get the dimension of the simplicial complex.
     *
     * Return the maximum dimension of the simplices in the simplicial complex.
     * 
     * @return IndexT 
     */
    [[nodiscard]] IndexT dim() const;

  protected:
    virtual std::string_view get_type() const override;

  private:
    AbstractSimplicialComplex m_asc;
    AttributeCollection       m_vertex_attributes;
    AttributeCollection       m_edge_attributes;
    AttributeCollection       m_triangle_attributes;
    AttributeCollection       m_tetrahedron_attributes;
};
}  // namespace uipc::geometry