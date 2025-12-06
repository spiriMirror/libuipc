#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/constraints/affine_body_external_body_force_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <muda/ext/eigen/atomic.h>

namespace uipc::backend::cuda
{
/**
 * @brief Get external forces from ExternalForceConstraint and apply them to Affine Bodies
 *
 * This reporter add forces to Affine Bodies in the AffineBodyDynamics system.
 *
 * This is the "body force" implementation - forces are applied directly to bodies.
 * Future implementations like AffineBodyExternalVertexForce may apply forces to vertices.
 */
class AffineBodyExternalBodyForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 UID = 666;  // Same UID as ExternalForceConstraint

    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    SimSystemSlot<AffineBodyDynamics>                    affine_body_dynamics;
    SimSystemSlot<AffineBodyExternalBodyForceConstraint> constraint;

    virtual void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        constraint           = require<AffineBodyExternalBodyForceConstraint>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override
    {
        // Nothing to do
    }

    void do_step(ExternalForceInfo& info) override
    {
        SizeT force_count = constraint->forces().size();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(force_count,
                   [forces   = info.external_forces().viewer().name("forces"),
                    body_ids = constraint->body_ids().viewer().name("body_ids"),
                    body_forces = constraint->forces().viewer().name(
                        "body_forces")] __device__(int i) mutable
                   {
                       // Scatter add the external forces to the corresponding bodies
                       auto body_id = body_ids(i);
                       eigen::atomic_add(forces(body_id), body_forces(i));
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyExternalBodyForce);
}  // namespace uipc::backend::cuda
