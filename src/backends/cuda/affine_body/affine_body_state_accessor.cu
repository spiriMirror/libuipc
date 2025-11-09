#include <sim_system.h>
#include <affine_body/affine_body_state_accessor_feature.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_vertex_reporter.h>

namespace uipc::backend::cuda
{
// A SimSystem to add AffineBodyStateAccessorFeature to the engine
class AffineBodyStateAccessor final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    virtual void do_build() override
    {
        auto& affine_body_dynamics        = require<AffineBodyDynamics>();
        auto& affine_body_vertex_reporter = require<AffineBodyVertexReporter>();

        // Register the AffineBodyStateAccessorFeature
        auto overrider = std::make_shared<AffineBodyStateAccessorFeatureOverrider>(
            affine_body_dynamics, affine_body_vertex_reporter);

        auto feature = std::make_shared<core::AffineBodyStateAccessorFeature>(overrider);
        features().insert(feature);
    }
};

REGISTER_SIM_SYSTEM(AffineBodyStateAccessor);
}  // namespace uipc::backend::cuda
