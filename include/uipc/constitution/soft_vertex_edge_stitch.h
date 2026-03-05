#pragma once
#include <uipc/constitution/inter_primitive_constitution.h>
#include <uipc/constitution/elastic_moduli.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/simplicial_complex_slot.h>
#include <uipc/common/span.h>
#include <uipc/common/unit.h>

namespace uipc::constitution
{
/**
 * @brief Inter-primitive constitution: (vertex, edge) pairs form triangles with StVK membrane energy.
 * UID = 29.
 */
class UIPC_CONSTITUTION_API SoftVertexEdgeStitch final : public InterPrimitiveConstitution
{
    using Base = InterPrimitiveConstitution;

  public:
    using SlotTuple =
        std::tuple<S<geometry::SimplicialComplexSlot>, S<geometry::SimplicialComplexSlot>>;

    SoftVertexEdgeStitch(const Json& config = default_config());

    /**
     * @brief Create stitch geometry from explicit (vertex_index, edge_index) pairs.
     */
    geometry::Geometry create_geometry(
        const SlotTuple&     aim_geo_slots,
        const SlotTuple&     rest_geo_slots,
        span<const Vector2i> stitched_vert_edge_ids,
        const ElasticModuli2D& moduli = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
        Float thickness             = 0.001,
        Float min_separate_distance = 0.001) const;

    /**
     * @brief Create stitch geometry from the Geometry returned by closest_vertex_edge_pairs.
     */
    geometry::Geometry create_geometry(
        const SlotTuple&          aim_geo_slots,
        const SlotTuple&          rest_geo_slots,
        const geometry::Geometry& pair_geometry,
        const ElasticModuli2D& moduli = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
        Float thickness             = 0.001,
        Float min_separate_distance = 0.001) const;

    static Json default_config();

  private:
    U64  get_uid() const noexcept override;
    Json m_config;
};
}  // namespace uipc::constitution
