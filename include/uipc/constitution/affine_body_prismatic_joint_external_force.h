#pragma once
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyPrismaticJointExternalForce final : public Constraint
{
    using Base = Constraint;

  public:
    AffineBodyPrismaticJointExternalForce(const Json& config = default_config());

    ~AffineBodyPrismaticJointExternalForce() override;

    /**
     * @brief Apply uniform external force along the prismatic joint axis to all joints.
     *
     * Must be applied AFTER AffineBodyPrismaticJoint (constitution_uid == 20).
     * The force is a scalar: +f*t applied to body_i, -f*t applied to body_j,
     * where t is the joint tangent direction.
     *
     * @param sc The simplicial complex containing the edges representing the joints.
     * @param force The scalar force value applied to all joints.
     */
    void apply_to(geometry::SimplicialComplex& sc, Float force = Float{0});

    /**
     * @brief Apply per-joint external forces along the prismatic joint axis.
     *
     * Must be applied AFTER AffineBodyPrismaticJoint (constitution_uid == 20).
     *
     * @param sc The simplicial complex containing the edges representing the joints.
     * @param forces Per-joint scalar force values (one per edge).
     */
    void apply_to(geometry::SimplicialComplex& sc, span<Float> forces);

    static Json default_config();

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
