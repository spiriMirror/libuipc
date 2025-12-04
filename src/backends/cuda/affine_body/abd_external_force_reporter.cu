#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
/**
 * @brief Reporter that computes M^{-1} * F for external forces
 *
 * This reporter reads raw forces from body_id_to_external_force_raw (set by constraints)
 * and computes the acceleration M^{-1} * F, storing it in body_id_to_external_force.
 */
class ABDExternalForceReporter final : public AffineBodyExternalForceReporter
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
        // At this point, constraints have already updated body_id_to_external_force_raw
        // Now we compute M^{-1} * F and store in body_id_to_external_force

        auto forces_raw = affine_body_dynamics->body_external_forces_raw();
        auto forces     = affine_body_dynamics->body_external_forces();
        auto masses_inv = affine_body_dynamics->body_mass_invs();

        SizeT body_count = forces_raw.size();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_count,
                   [forces_raw = forces_raw.cviewer().name("forces_raw"),
                    forces     = forces.viewer().name("forces"),
                    masses_inv = masses_inv.cviewer().name("masses_inv")] __device__(int i) mutable
                   {
                       const auto& F     = forces_raw(i);
                       const auto& M_inv = masses_inv(i);

                       // Compute acceleration: a = M^{-1} * F (like gravity)
                       forces(i) = M_inv * F;
                   });
    }
};

REGISTER_SIM_SYSTEM(ABDExternalForceReporter);
}  // namespace uipc::backend::cuda
