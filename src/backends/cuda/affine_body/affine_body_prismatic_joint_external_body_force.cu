#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/constraints/affine_body_prismatic_joint_external_body_force_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/constitutions/affine_body_prismatic_joint_function.h>
#include <time_integrator/time_integrator.h>
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

class AffineBodyPrismaticJointExternalForceTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    SimSystemSlot<AffineBodyPrismaticJointExternalBodyForceConstraint> constraint;
    SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

    void do_init(InitInfo& info) override {}

    void do_build(BuildInfo& info) override
    {
        constraint          = require<AffineBodyPrismaticJointExternalBodyForceConstraint>();
        affine_body_dynamics = require<AffineBodyDynamics>();
    }

    void do_predict_dof(PredictDofInfo& info) override {}

    void do_update_state(UpdateVelocityInfo& info) override
    {
        using namespace muda;
        namespace DPJ = sym::affine_body_driving_prismatic_joint;

        SizeT N = constraint->body_ids().size();
        if(N == 0)
            return;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [body_ids       = constraint->body_ids().cviewer().name("body_ids"),
                    rest_positions = constraint->rest_positions().cviewer().name("rest_positions"),
                    rest_tangents  = constraint->rest_tangents().cviewer().name("rest_tangents"),
                    init_distances = constraint->init_distances().cviewer().name("init_distances"),
                    current_distances = constraint->current_distances().viewer().name("current_distances"),
                    qs = affine_body_dynamics->qs().cviewer().name("qs")] __device__(int I)
                   {
                       Vector2i bids = body_ids(I);

                       Vector12 q_i = qs(bids(0));
                       Vector12 q_j = qs(bids(1));

                       const Vector6& C_bar = rest_positions(I);
                       const Vector6& t_bar = rest_tangents(I);

                       Vector9 F01_q;
                       DPJ::F01_q<Float>(F01_q,
                                         C_bar.segment<3>(0),
                                         t_bar.segment<3>(0),
                                         q_i,
                                         C_bar.segment<3>(3),
                                         t_bar.segment<3>(3),
                                         q_j);

                       Float distance;
                       DPJ::Distance<Float>(distance, F01_q);

                       current_distances(I) = distance - init_distances(I);
                   });
    }
};
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalForceTimeIntegrator);

}  // namespace uipc::backend::cuda
