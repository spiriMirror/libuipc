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

    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  Float strength_ratio = Float{100});

    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                  span<IndexT>                             l_instance_id,
                  span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                  span<IndexT>                             r_instance_id,
                  span<Float>                              strength_ratio);

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
