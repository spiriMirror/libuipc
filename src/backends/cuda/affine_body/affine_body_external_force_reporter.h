#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_external_force_manager.h>

namespace uipc::backend::cuda
{
class AffineBodyExternalForceManager;

/**
 * @brief Base class for affine body external force reporters
 *
 * Reporters are responsible for computing M^{-1} * F (acceleration from force)
 * and updating body_id_to_external_force_acc in AffineBodyDynamics.
 *
 * Specific implementations:
 * - AffineBodyExternalBodyForce: Applies force directly to body
 * - AffineBodyExternalVertexForce (future): Applies force to vertices, sums to body
 *
 * Lifecycle:
 * - do_init(): Called once during initialization
 * - do_step(): Called every frame AFTER constraints have updated body_id_to_external_force
 */
class AffineBodyExternalForceReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    U64 uid() const noexcept;

    using ExternalForceInfo = AffineBodyExternalForceManager::ExternalForceInfo;

  protected:
    virtual void do_build(BuildInfo& info)        = 0;
    virtual U64  get_uid() const noexcept         = 0;
    virtual void do_init()                        = 0;
    virtual void do_step(ExternalForceInfo& info) = 0;

  private:
    friend class AffineBodyExternalForceManager;
    virtual void do_build() override final;

    void init();  // only be called by AffineBodyExternalForceManager
    void step(ExternalForceInfo& info);  // only be called by AffineBodyExternalForceManager

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
