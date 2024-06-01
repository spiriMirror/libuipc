#pragma once
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/simplicial_complex.h>
namespace uipc::geometry
{
template <>
class GeometrySlot<SimplicialComplex> : public IGeometrySlot
{
  private:
    using VertexAttributes    = typename SimplicialComplex::VertexAttributes;
    using CVertexAttributes   = typename SimplicialComplex::CVertexAttributes;
    using EdgeAttributes      = typename SimplicialComplex::EdgeAttributes;
    using CEdgeAttributes     = typename SimplicialComplex::CEdgeAttributes;
    using TriangleAttributes  = typename SimplicialComplex::TriangleAttributes;
    using CTriangleAttributes = typename SimplicialComplex::CTriangleAttributes;
    using TetrahedronAttributes = typename SimplicialComplex::TetrahedronAttributes;
    using CTetrahedronAttributes = typename SimplicialComplex::CTetrahedronAttributes;

  public:
    GeometrySlot(IndexT id, const SimplicialComplex& simplicial_complex) noexcept;

    GeometrySlot(const GeometrySlot&)            = delete;
    GeometrySlot(GeometrySlot&&)                 = delete;
    GeometrySlot& operator=(const GeometrySlot&) = delete;
    GeometrySlot& operator=(GeometrySlot&&)      = delete;

    /**
     * @brief Get the positions of the vertices
     * 
     * @return AttributeSlot<Vector3>& 
     */
    [[nodiscard]] AttributeSlot<Vector3>& positions() noexcept;

    /**
     * @brief A short cut to get the positions of the vertices
     * 
     * @return const AttributeSlot<Vector3>& 
     */
    [[nodiscard]] const AttributeSlot<Vector3>& positions() const noexcept;

    /**
     * @brief A wrapper of the vertices and its attributes of the simplicial complex.
     * 
     * @return VertexAttributes 
     */
    [[nodiscard]] VertexAttributes  vertices() noexcept;
    [[nodiscard]] CVertexAttributes vertices() const noexcept;

    /**
     * @brief A wrapper of the edges and its attributes of the simplicial complex.
     * 
     * @return EdgeAttributes 
     */
    [[nodiscard]] EdgeAttributes  edges() noexcept;
    [[nodiscard]] CEdgeAttributes edges() const noexcept;


    /**
     * @brief  A wrapper of the triangles and its attributes of the simplicial complex.
     * 
     * @return TriangleAttributes 
     */
    [[nodiscard]] TriangleAttributes  triangles() noexcept;
    [[nodiscard]] CTriangleAttributes triangles() const noexcept;
    /**
     * @brief A wrapper of the tetrahedra and its attributes of the simplicial complex.
     * 
     * @return TetrahedronAttributes 
     */
    [[nodiscard]] TetrahedronAttributes  tetrahedra() noexcept;
    [[nodiscard]] CTetrahedronAttributes tetrahedra() const noexcept;

    /**
     * @brief Get the dimension of the simplicial complex.
     *
     * Return the maximum dimension of the simplices in the simplicial complex.
     * 
     * @return IndexT 
     */
    [[nodiscard]] IndexT dim() const noexcept;

    SimplicialComplex&       geometry() noexcept;
    const SimplicialComplex& geometry() const noexcept;

  protected:
    virtual IndexT get_id() const noexcept override;
    virtual Geometry& get_geometry() noexcept override;
    virtual const Geometry& get_geometry() const noexcept override;

  private:
    IndexT            m_id;
    SimplicialComplex m_simplicial_complex;
};

using SimplicialComplexSlot = GeometrySlot<SimplicialComplex>;
}  // namespace uipc::geometry
