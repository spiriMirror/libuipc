#include <numbers>
#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/constraints/affine_body_revolute_joint_external_body_force_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>
#include <time_integrator/time_integrator.h>
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
                           F_i.segment<3>(0) = -tau * e_world_i.cross(r_i) / r_sq_i;
                       }

                       Vector12 F_j = Vector12::Zero();
                       if(r_sq_j > eps)
                       {
                           F_j.segment<3>(0) = tau * e_world_j.cross(r_j) / r_sq_j;
                       }

                       eigen::atomic_add(external_forces(bids(0)), F_i);
                       eigen::atomic_add(external_forces(bids(1)), F_j);
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalBodyForce);

class AffineBodyRevoluteJointExternalForceTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<AffineBodyRevoluteJointExternalBodyForceConstraint> constraint;
    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    void do_init(InitInfo& info) override {}

    void do_build(BuildInfo& info) override
    {
        constraint = require<AffineBodyRevoluteJointExternalBodyForceConstraint>();
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_predict_dof(PredictDofInfo& info) override {}

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace DRJ = sym::affine_body_driving_revolute_joint;

        SizeT N = constraint->body_ids().size();
        if(N == 0)
            return;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [body_ids = constraint->body_ids().cviewer().name("body_ids"),
                    rest_positions = constraint->rest_positions().cviewer().name("rest_positions"),
                    init_angles = constraint->init_angles().cviewer().name("init_angles"),
                    current_angles = constraint->current_angles().viewer().name("current_angles"),
                    qs = affine_body_dynamics->qs().cviewer().name("qs"),
                    PI = std::numbers::pi] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       const Vector12& X_bar = rest_positions(I);

                       Vector3 x0_bar = X_bar.segment<3>(0);
                       Vector3 x1_bar = X_bar.segment<3>(3);
                       Vector3 x2_bar = X_bar.segment<3>(6);
                       Vector3 x3_bar = X_bar.segment<3>(9);

                       Vector3 h_bar_i = (x1_bar - x0_bar) * 0.5;
                       Vector3 h_bar_j = (x3_bar - x2_bar) * 0.5;

                       Vector3 e_bar_i = h_bar_i.normalized();
                       Vector3 e_bar_j = h_bar_j.normalized();

                       auto toNormal = [](const Vector3& W) -> Vector3
                       {
                           Vector3 ref = abs(W.dot(Vector3(1, 0, 0))) < 0.99 ?
                                             Vector3(1, 0, 0) :
                                             Vector3(0, 1, 0);
                           Vector3 U   = ref.cross(W).normalized();
                           return W.cross(U).normalized();
                       };

                       Vector3 n_bar_i = toNormal(e_bar_i);
                       Vector3 v_bar_i = n_bar_i.cross(e_bar_i).normalized();
                       Vector3 n_bar_j = toNormal(e_bar_j);
                       Vector3 v_bar_j = n_bar_j.cross(e_bar_j).normalized();

                       Vector12 F01_q;
                       DRJ::F01_q<Float>(F01_q, v_bar_i, n_bar_i, q_i, v_bar_j, n_bar_j, q_j);

                       Float curr_angle;
                       DRJ::currAngle<Float>(curr_angle, F01_q);

                       auto map2range = [=](Float angle) -> Float
                       {
                           if(angle > PI)
                               angle -= 2 * PI;
                           else if(angle < -PI)
                               angle += 2 * PI;
                           return angle;
                       };

                       current_angles(I) = map2range(curr_angle - init_angles(I));
                   });
    }
};
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalForceTimeIntegrator);

}  // namespace uipc::backend::cuda
