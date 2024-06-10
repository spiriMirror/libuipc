#pragma once
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/simplicial_complex.h>
namespace uipc::geometry
{
template <>
class GeometrySlotT<SimplicialComplex> : public GeometrySlot
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
    GeometrySlotT(IndexT id, const SimplicialComplex& simplicial_complex) noexcept;

    GeometrySlotT(const GeometrySlotT&)            = delete;
    GeometrySlotT(GeometrySlotT&&)                 = delete;
    GeometrySlotT& operator=(const GeometrySlotT&) = delete;
    GeometrySlotT& operator=(GeometrySlotT&&)      = delete;

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
    virtual Geometry&       get_geometry() noexcept override;
    virtual const Geometry& get_geometry() const noexcept override;

  private:
    SimplicialComplex m_simplicial_complex;
};


UIPC_EXTERN_TEMPLATE_CLASS GeometrySlotT<SimplicialComplex>;

using SimplicialComplexSlot = GeometrySlotT<SimplicialComplex>;
}  // namespace uipc::geometry
