#pragma once
#include <uipc/common/type_define.h>
#include <uipc/constitution/constraint.h>
#include <uipc/geometry/geometry_slot.h>

namespace uipc::constitution
{
/**
 * @brief External articulation constraint for incorporating external kinetic energy into the IPC system.
 * 
 * Controls motion of affine bodies by prescribing articulation DOFs (joint positions, angles, etc.).
 * Incorporates kinetic energy term: K = 1/2(delta_theta - delta_theta_tilde)^T M^t (delta_theta - delta_theta_tilde)
 * where delta_theta is variational joint DOF, delta_theta_tilde is predicted variational joint DOF,
 * and M^t is effective mass matrix at previous time step.
 * 
 * Users must provide M^t and delta_theta_tilde through animator callback. Works with AffineBodyRevoluteJoint
 * and AffineBodyPrismaticJoint to constrain affine bodies to follow articulation motion.
 */
class UIPC_CONSTITUTION_API ExternalArticulationConstraint final : public Constraint
{
    using Base = Constraint;

  public:
    /**
     * @brief Construct an ExternalArticulationConstraint.
     * 
     * @param config Configuration dictionary (optional, uses default if not provided).
     */
    ExternalArticulationConstraint(const Json& config = default_config()) noexcept;

    /**
     * @brief Destructor.
     */
    ~ExternalArticulationConstraint();

    /**
     * @brief Create geometry for external articulation joints.
     * 
     * Creates geometry structure with joint attributes (geo_id, index, delta_theta_tilde, delta_theta)
     * and joint-joint mass attributes. All joints use index 0 by default.
     * 
     * @param joint_geos Geometry slots containing joint information.
     * @return geometry::Geometry Created geometry with joint constraint data.
     */
    [[nodiscard]] geometry::Geometry create_geometry(span<S<const geometry::GeometrySlot>> joint_geos) const;

    /**
     * @brief Create geometry for external articulation joints with specified indices.
     * 
     * Same as single-parameter version but allows selecting specific joints from each geometry slot.
     * 
     * @param joint_geos Geometry slots containing joint information.
     * @param indices Indices specifying which joint to use from each slot. Must match joint_geos size.
     * @return geometry::Geometry Created geometry with joint constraint data.
     */
    [[nodiscard]] geometry::Geometry create_geometry(span<S<const geometry::GeometrySlot>> joint_geos,
                                                     span<IndexT> indices) const;

    /**
     * @brief Get the default configuration for ExternalArticulationConstraint.
     * 
     * @return Json Default configuration dictionary (currently empty).
     */
    static Json default_config();

  protected:
    /**
     * @brief Get the unique identifier for this constraint type.
     * 
     * @return U64 The constraint UID.
     */
    virtual U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution