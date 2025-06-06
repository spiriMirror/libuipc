#pragma once
#include <uipc/constitution/inter_affine_body_constitution.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyRevoluteJoint final : public InterAffineBodyConstitution
{
  public:
    using SlotTuple =
        std::tuple<S<geometry::SimplicialComplexSlot>, S<geometry::SimplicialComplexSlot>>;

    static Json default_config();

    AffineBodyRevoluteJoint(const Json& config = default_config());

    virtual ~AffineBodyRevoluteJoint();

    void apply_to(geometry::SimplicialComplex& edges,
                  span<SlotTuple>              geo_slots,
                  Float                        strength_ratio = Float{100});

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
