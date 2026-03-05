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
 * @brief Inter-primitive constitution: (vertex, triangle) pairs form tetrahedra with StableNeoHookean energy.
 * UID = 29.
 */
class UIPC_CONSTITUTION_API SoftVertexTriangleStitch final : public InterPrimitiveConstitution
{
    using Base = InterPrimitiveConstitution;

  public:
    using SlotTuple =
        std::tuple<S<geometry::SimplicialComplexSlot>, S<geometry::SimplicialComplexSlot>>;

    SoftVertexTriangleStitch(const Json& config = default_config());

    /**
     * @brief Create stitch geometry from explicit (vertex_index, triangle_index) pairs.
     * When a tet is degenerate, rest vertex is offset by min_separate_distance (default 1 mm).
     */
    geometry::Geometry create_geometry(
        const SlotTuple&     aim_geo_slots,
        const SlotTuple&     rest_geo_slots,
        span<const Vector2i> stitched_vert_tri_ids,
        const ElasticModuli& moduli = ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
        Float min_separate_distance = 0.001) const;

    /**
     * @brief Create stitch geometry from the Geometry returned by closest_vertex_triangle_pairs.
     * When a tet is degenerate, uses min_separate_distance (default 1 mm).
     */
    geometry::Geometry create_geometry(
        const SlotTuple&          aim_geo_slots,
        const SlotTuple&          rest_geo_slots,
        const geometry::Geometry& pair_geometry,
        const ElasticModuli& moduli = ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
        Float min_separate_distance = 0.001) const;

    static Json default_config();

  private:
    U64  get_uid() const noexcept override;
    Json m_config;
};
}  // namespace uipc::constitution
