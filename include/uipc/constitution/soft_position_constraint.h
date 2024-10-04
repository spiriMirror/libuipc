#pragma once
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/json.h>
namespace uipc::constitution
{
class UIPC_CONSTITUTION_API SoftPositionConstraint final : public Constraint
{
    using Base = Constraint;

  public:
    SoftPositionConstraint(const Json& config = default_config()) noexcept;

    /**
     * @brief Apply the constraint to the simplicial complex vertices.
     * 
     * @param sc The simplicial complex to apply the constraint to.
     * @param strength_rate The strength of the constraint will be `strength_rate * vertex_mass`.
     * @param is_kinematic If the vertices' kinetic energy is not considered.
     */
    void apply_to(geometry::SimplicialComplex& sc, Float strength_rate = 100.0) const;

    static Json default_config();

  protected:
    U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
