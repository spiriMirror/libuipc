#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class ExternalForceReporter;

/**
 * @brief Global manager for external forces (agnostic to dynamics type)
 *
 * This system manages the lifecycle of external force reporters.
 * It provides a new lifecycle phase that happens after GlobalAnimator::step()
 * but before TimeIntegrator::do_predict_dof().
 *
 * The manager itself does not know about specific dynamics types (AffineBody, FEM, etc).
 * It only manages ExternalForceReporter instances.
 */
class GlobalExternalForceManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    friend class SimEngine;
    void init();   // only be called by SimEngine
    void clear();  // only be called by SimEngine - clear buffers before constraints
    void step();   // only be called by SimEngine - compute accelerations after constraints

    friend class ExternalForceReporter;
    void register_reporter(ExternalForceReporter* reporter);

    virtual void do_build() override;

    SimSystemSlotCollection<ExternalForceReporter> m_reporters;
};
}  // namespace uipc::backend::cuda
