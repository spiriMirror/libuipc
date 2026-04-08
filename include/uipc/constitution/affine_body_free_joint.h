#pragma once
#include <uipc/constitution/inter_affine_body_constitution.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyFreeJoint final : public InterAffineBodyConstitution
{
  public:
    static Json default_config();

    AffineBodyFreeJoint(const Json& config = default_config());

    virtual ~AffineBodyFreeJoint();

    [[nodiscard]] geometry::SimplicialComplex create_geometry(
        span<S<geometry::SimplicialComplexSlot>> geo_slots, span<IndexT> instance_ids);

    void apply_to(geometry::SimplicialComplex&             sc,
                  span<S<geometry::SimplicialComplexSlot>> geo_slots,
                  span<IndexT>                             instance_ids);

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
