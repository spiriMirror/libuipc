#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/constraints/affine_body_prismatic_joint_external_body_force_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <muda/ext/eigen/atomic.h>

namespace uipc::backend::cuda
{
class AffineBodyPrismaticJointExternalBodyForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 UID = 667;

    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
    SimSystemSlot<AffineBodyPrismaticJointExternalBodyForceConstraint> constraint;

    virtual void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        constraint = require<AffineBodyPrismaticJointExternalBodyForceConstraint>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override
    {
        // Nothing to do
    }

    void do_step(ExternalForceInfo& info) override
    {
        SizeT force_count = constraint->forces().size();
        if(force_count == 0)
            return;

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(force_count,
                   [external_forces = info.external_forces().viewer().name("external_forces"),
                    body_ids = constraint->body_ids().viewer().name("body_ids"),
                    forces   = constraint->forces().viewer().name("forces"),
                    rest_tangents = constraint->rest_tangents().viewer().name("rest_tangents"),
                    qs = affine_body_dynamics->qs().cviewer().name("qs")] __device__(int i) mutable
                   {
                       Vector2i bids = body_ids(i);
                       Float    f    = forces(i);

                       const Vector6& t_bar = rest_tangents(i);

                       // Get body state q_i to extract rotation
                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       ABDJacobi JT[2] = {ABDJacobi{t_bar.segment<3>(0)},
                                          ABDJacobi{t_bar.segment<3>(3)}};
                       Vector3   t_i   = JT[0].vec_x(q_i);
                       Vector3   t_j   = JT[1].vec_x(q_j);

                       // Build 12D force vectors
                       // F = [fx, fy, fz, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                       Vector12 F_i      = Vector12::Zero();
                       F_i.segment<3>(0) = -f * t_i;

                       Vector12 F_j      = Vector12::Zero();
                       F_j.segment<3>(0) = f * t_j;

                       // Scatter add to external forces
                       eigen::atomic_add(external_forces(bids(0)), F_i);
                       eigen::atomic_add(external_forces(bids(1)), F_j);
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalBodyForce);
}  // namespace uipc::backend::cuda
