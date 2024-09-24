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

    SimplicialComplex&       geometry() noexcept;
    const SimplicialComplex& geometry() const noexcept;

  protected:
    virtual Geometry&       get_geometry() noexcept override;
    virtual const Geometry& get_geometry() const noexcept override;

  private:
    SimplicialComplex m_simplicial_complex;
};


UIPC_CORE_EXPORT_TEMPLATE_CLASS(GeometrySlotT<SimplicialComplex>);
UIPC_CORE_EXPORT_TEMPLATE_CLASS(std::shared_ptr<GeometrySlotT<SimplicialComplex>>);

using SimplicialComplexSlot = GeometrySlotT<SimplicialComplex>;
}  // namespace uipc::geometry