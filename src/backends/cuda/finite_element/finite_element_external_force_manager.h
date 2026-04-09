#pragma once
#include <external_force/external_force_reporter.h>
#include <muda/buffer/buffer_view.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;
class FiniteElementExternalForceReporter;

/**
 * @brief Manager for finite element external forces
 *
 * This reporter manages all FEM-specific external force reporters.
 * It is responsible for:
 * 1. Clearing the external force buffer before constraints update
 * 2. Collecting external forces from all registered reporters
 * 3. Computing the external accelerations after all forces have been applied
 *
 * Hierarchy:
 * - FEMExternalForceManager (this class, derives from ExternalForceReporter)
 *   - Manages FiniteElementExternalForceReporter instances
 *     - FiniteElementExternalVertexForce (applies force to vertices)
 */
class FEMExternalForceManager final : public ExternalForceReporter
{
  public:
    using ExternalForceReporter::ExternalForceReporter;

    static constexpr U64 UID = 671;

    class Impl
    {
      public:
        FiniteElementMethod* finite_element_method = nullptr;
        SimSystemSlotCollection<FiniteElementExternalForceReporter> m_reporters;

        void clear();
        void step();
    };

    class ExternalForceInfo
    {
      public:
        ExternalForceInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }
        muda::BufferView<Vector3> external_forces() noexcept;

      private:
        friend class FEMExternalForceManager;
        Impl* m_impl = nullptr;
    };

  private:
    virtual void do_build(BuildInfo& info) override;
    virtual U64  get_uid() const noexcept override { return UID; }
    virtual void do_init() override;
    virtual void do_clear() override;
    virtual void do_step() override;

    friend class FiniteElementExternalForceReporter;
    void register_reporter(FiniteElementExternalForceReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
