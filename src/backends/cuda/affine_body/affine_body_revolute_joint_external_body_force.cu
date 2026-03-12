#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/constraints/affine_body_revolute_joint_external_body_force_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <muda/ext/eigen/atomic.h>

namespace uipc::backend::cuda
{
class AffineBodyRevoluteJointExternalBodyForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 UID = 668;

    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
    SimSystemSlot<AffineBodyRevoluteJointExternalBodyForceConstraint> constraint;

    virtual void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        constraint = require<AffineBodyRevoluteJointExternalBodyForceConstraint>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override {}

    void do_step(ExternalForceInfo& info) override
    {
        SizeT torque_count = constraint->torques().size();
        if(torque_count == 0)
            return;

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(torque_count,
                   [external_forces = info.external_forces().viewer().name("external_forces"),
                    body_ids = constraint->body_ids().viewer().name("body_ids"),
                    torques  = constraint->torques().viewer().name("torques"),
                    rest_positions = constraint->rest_positions().viewer().name("rest_positions"),
                    qs = affine_body_dynamics->qs().cviewer().name("qs")] __device__(int i) mutable
                   {
                       Vector2i bids = body_ids(i);
                       Float    tau  = torques(i);

                       const Vector12& X_bar = rest_positions(i);

                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       Vector3 x0_bar = X_bar.segment<3>(0);
                       Vector3 x1_bar = X_bar.segment<3>(3);
                       Vector3 x2_bar = X_bar.segment<3>(6);
                       Vector3 x3_bar = X_bar.segment<3>(9);

                       // Axis direction in world frame
                       Vector3 e_world_i =
                           ABDJacobi{x1_bar - x0_bar}.vec_x(q_i).normalized();
                       Vector3 e_world_j =
                           ABDJacobi{x3_bar - x2_bar}.vec_x(q_j).normalized();

                       // Vector from axis point x0 to center of mass (x_bar=0) in world:
                       //   c - x0_world = c - (c + A * x0_bar) = -A * x0_bar
                       Vector3 d_i = -ABDJacobi{x0_bar}.vec_x(q_i);
                       Vector3 d_j = -ABDJacobi{x2_bar}.vec_x(q_j);

                       // Project center of mass onto axis, lever arm is the
                       // perpendicular component: r = d - (d·e)*e
                       Vector3 r_i = d_i - d_i.dot(e_world_i) * e_world_i;
                       Vector3 r_j = d_j - d_j.dot(e_world_j) * e_world_j;

                       Float r_sq_i = r_i.squaredNorm();
                       Float r_sq_j = r_j.squaredNorm();

                       // Tangential force at center of mass:
                       //   F = tau * (e × r) / |r|^2
                       // Body_i receives -tau (reaction), body_j receives +tau.

                       constexpr Float eps = 1e-12;

                       Vector12 F_i = Vector12::Zero();
                       if(r_sq_i > eps)
                       {
                           F_i.segment<3>(0) = tau * e_world_i.cross(r_i) / r_sq_i;
                       }

                       Vector12 F_j = Vector12::Zero();
                       if(r_sq_j > eps)
                       {
                           F_j.segment<3>(0) = -tau * e_world_j.cross(r_j) / r_sq_j;
                       }

                       eigen::atomic_add(external_forces(bids(0)), F_i);
                       eigen::atomic_add(external_forces(bids(1)), F_j);
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalBodyForce);
}  // namespace uipc::backend::cuda
