#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class GlobalExternalForceManager;

/**
 * @brief Base class for external force reporters
 *
 * Reporters are responsible for computing M^{-1} * F (acceleration from force)
 * and updating body_id_to_external_force in AffineBodyDynamics.
 *
 * Lifecycle:
 * - do_init(): Called once during initialization
 * - do_step(): Called every frame AFTER constraints have updated body_id_to_external_force_raw
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

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual U64  get_uid() const noexcept  = 0;
    virtual void do_init()                 = 0;
    virtual void do_step()                 = 0;

  private:
    friend class GlobalExternalForceManager;
    virtual void do_build() override final;

    void init();
    void step();

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
