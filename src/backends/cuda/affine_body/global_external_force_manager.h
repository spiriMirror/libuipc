#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class AffineBodyExternalForceReporter;

/**
 * @brief Global manager for external forces applied to affine bodies
 *
 * This system manages the lifecycle of external force reporters.
 * It provides a new lifecycle phase that happens after GlobalAnimator::step()
 * but before ABDTimeIntegrator::do_predict_dof().
 *
 * Lifecycle:
 * 1. GlobalAnimator::step()
 *    └─> Constraints read external_force from geometries and store to body_id_to_external_force_raw
 *
 * 2. GlobalExternalForceManager::step() [NEW LIFECYCLE!]
 *    └─> Reporters compute M^{-1} * F and update body_id_to_external_force
 *
 * 3. ABDTimeIntegrator::do_predict_dof()
 *    └─> Uses body_id_to_external_force (acceleration)
 */
class GlobalExternalForceManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    friend class SimEngine;
    void init();   // only be called by SimEngine
    void step();   // only be called by SimEngine

    friend class AffineBodyExternalForceReporter;
    void register_reporter(AffineBodyExternalForceReporter* reporter);

    virtual void do_build() override;

    SimSystemSlotCollection<AffineBodyExternalForceReporter> m_reporters;
};
}  // namespace uipc::backend::cuda
