#pragma once
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/json.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API SoftTransformConstraint final : public Constraint
{
    using Base = Constraint;

  public:
    SoftTransformConstraint(const Json& config = default_config()) noexcept;

    /**
     * @brief Apply the constraint to the simplicial complex instances.
     * 
     * @param sc The simplicial complex to apply the constraint to.
     * @param strength_ratio The strength ratio of the constraint, strength_ratio[0] is the strength of the translation,
     * strength_ratio[1] is the strength of the rotation.
     */
    void apply_to(geometry::SimplicialComplex& sc, const Vector2& strength_ratio) const;

    static Json default_config();

  protected:
    U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution