#pragma once
#include <uipc/constitution/inter_affine_body_constitution.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodySphericalJoint final : public InterAffineBodyConstitution
{
  public:
    static Json default_config();

    AffineBodySphericalJoint(const Json& config = default_config());

    virtual ~AffineBodySphericalJoint();

    // Simplified API: anchor at body1's (right body's) origin in local space
    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  Float strength_ratio = Float{100});

    // Full API: anchor specified as r_local_pos in body1's (right body's) local frame
    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<IndexT>                             l_instance_id,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  span<IndexT>                             r_instance_id,
                  span<Vector3>                            r_local_pos,
                  span<Float>                              strength_ratio);

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
