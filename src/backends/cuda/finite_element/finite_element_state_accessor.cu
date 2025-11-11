#include <sim_system.h>
#include <finite_element/finite_element_state_accessor_feature.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_vertex_reporter.h>

namespace uipc::backend::cuda
{
// A SimSystem to add FiniteElementStateAccessorFeature to the engine
class FiniteElementStateAccessor final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    virtual void do_build() override
    {
        auto& finite_element_method = require<FiniteElementMethod>();
        auto& finite_element_vertex_reporter = require<FiniteElementVertexReporter>();

        // Register the FiniteElementStateAccessorFeature
        auto overrider = std::make_shared<FiniteElementStateAccessorFeatureOverrider>(
            finite_element_method, finite_element_vertex_reporter);
        auto feature = std::make_shared<core::FiniteElementStateAccessorFeature>(overrider);
        features().insert(feature);
    }
};

REGISTER_SIM_SYSTEM(FiniteElementStateAccessor);
}  // namespace uipc::backend::cuda
