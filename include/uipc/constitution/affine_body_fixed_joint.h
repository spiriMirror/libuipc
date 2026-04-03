#pragma once
#include <uipc/constitution/inter_affine_body_constitution.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyFixedJoint final : public InterAffineBodyConstitution
{
  public:
    static Json default_config();

    AffineBodyFixedJoint(const Json& config = default_config());

    virtual ~AffineBodyFixedJoint();

    /**
     * @brief Create fixed joint geometry with world-space positions derived from body transforms.
     *
     * Builds a vertex-based SimplicialComplex (1 vertex per joint).
     * Writes vertices.position as midpoint of the two body translations.
     * Does not write local position attributes.
     */
    [[nodiscard]] geometry::SimplicialComplex create_geometry(
        span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
        span<IndexT>                             l_instance_ids,
        span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
        span<IndexT>                             r_instance_ids,
        span<Float>                              strength_ratios);

    /**
     * @brief Create fixed joint geometry with local-space positions.
     *
     * Builds a vertex-based SimplicialComplex (1 vertex per joint).
     * Writes local position attributes (l_position, r_position) on vertices.
     */
    [[nodiscard]] geometry::SimplicialComplex create_geometry(
        span<const Vector3>                      l_positions,
        span<const Vector3>                      r_positions,
        span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
        span<IndexT>                             l_instance_ids,
        span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
        span<IndexT>                             r_instance_ids,
        span<Float>                              strength_ratios);

    /**
     * @brief Bind affine body instances to existing fixed joint geometry (single-instance mode).
     *
     * Pure binding: writes constitution UID, geometry IDs, instance IDs, and
     * strength ratios. Does NOT build geometry.
     */
    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  Float strength_ratio = Float{100});

    /**
     * @brief Bind affine body instances to existing fixed joint geometry (multi-instance mode).
     *
     * Pure binding: writes constitution UID, geometry IDs, instance IDs, and
     * strength ratios. Does NOT build geometry.
     */
    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<IndexT>                             l_instance_ids,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  span<IndexT>                             r_instance_ids,
                  span<Float>                              strength_ratios);

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
