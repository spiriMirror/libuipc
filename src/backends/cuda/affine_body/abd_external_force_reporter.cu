#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
/**
 * @brief Computes M^{-1} * F for external forces applied to affine bodies
 *
 * This reporter reads forces from body_id_to_external_force (set by constraints)
 * and computes the acceleration M^{-1} * F, storing it in body_id_to_external_force_acc.
 *
 * This is the "body force" implementation - forces are applied directly to bodies.
 * Future implementations like AffineBodyExternalVertexForce may apply forces to vertices.
 */
class AffineBodyExternalBodyForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 UID = 666;  // Same UID as ExternalForceConstraint

    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    AffineBodyDynamics* affine_body_dynamics = nullptr;

    virtual void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = &require<AffineBodyDynamics>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override
    {
        // Initial computation
        do_step();
    }

    void do_step() override
    {
        // At this point, constraints have already updated body_id_to_external_force
        // Now we compute M^{-1} * F and store in body_id_to_external_force_acc

        auto forces     = affine_body_dynamics->body_external_forces();
        auto force_accs = affine_body_dynamics->body_external_force_accs();
        auto masses_inv = affine_body_dynamics->body_mass_invs();

        SizeT body_count = forces.size();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_count,
                   [forces     = forces.cviewer().name("forces"),
                    force_accs = force_accs.viewer().name("force_accs"),
                    masses_inv = masses_inv.cviewer().name("masses_inv")] __device__(int i) mutable
                   {
                       const auto& F     = forces(i);
                       const auto& M_inv = masses_inv(i);

                       // Compute acceleration: a = M^{-1} * F (like gravity)
                       force_accs(i) = M_inv * F;
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyExternalBodyForce);
}  // namespace uipc::backend::cuda
