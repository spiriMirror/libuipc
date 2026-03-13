#pragma once
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyRevoluteJointExternalBodyForce final : public Constraint
{
    using Base = Constraint;

  public:
    AffineBodyRevoluteJointExternalBodyForce(const Json& config = default_config());

    ~AffineBodyRevoluteJointExternalBodyForce() override;

    /**
     * @brief Apply uniform external torque around the revolute joint axis to all joints.
     *
     * Must be applied AFTER AffineBodyRevoluteJoint (constitution_uid == 18).
     * The torque is a scalar: +τ applied to body_j, -τ applied to body_i,
     * where the axis is defined by the edge direction.
     *
     * @param sc The simplicial complex containing the edges representing the joints.
     * @param torque The scalar torque value applied to all joints.
     */
    void apply_to(geometry::SimplicialComplex& sc, Float torque = Float{0});

    /**
     * @brief Apply per-joint external torques around the revolute joint axis.
     *
     * Must be applied AFTER AffineBodyRevoluteJoint (constitution_uid == 18).
     *
     * @param sc The simplicial complex containing the edges representing the joints.
     * @param torques Per-joint scalar torque values (one per edge).
     */
    void apply_to(geometry::SimplicialComplex& sc, span<Float> torques);

    static Json default_config();

  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
