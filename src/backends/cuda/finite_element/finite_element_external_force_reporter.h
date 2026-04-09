#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_external_force_manager.h>

namespace uipc::backend::cuda
{
class FEMExternalForceManager;

/**
 * @brief Base class for finite element external force reporters
 *
 * Reporters scatter-add external forces into the per-vertex force buffer.
 * The manager then computes per-vertex accelerations a = F/m.
 *
 * Specific implementations:
 * - FiniteElementExternalVertexForce: Applies force directly to vertices
 *
 * Lifecycle:
 * - do_init(): Called once during initialization
 * - do_step(): Called every frame to scatter forces into the shared buffer
 */
class FiniteElementExternalForceReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    U64 uid() const noexcept;

    using ExternalForceInfo = FEMExternalForceManager::ExternalForceInfo;

  protected:
    virtual void do_build(BuildInfo& info)        = 0;
    virtual U64  get_uid() const noexcept         = 0;
    virtual void do_init()                        = 0;
    virtual void do_step(ExternalForceInfo& info) = 0;

  private:
    friend class FEMExternalForceManager;
    virtual void do_build() override final;

    void init();
    void step(ExternalForceInfo& info);

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
