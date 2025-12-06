#pragma once
#include <external_force/external_force_reporter.h>
#include <muda/buffer/buffer_view.h>

namespace uipc::backend::cuda
{
class AffineBodyDynamics;
class AffineBodyExternalForceReporter;

/**
 * @brief Manager for affine body external forces
 *
 * This reporter manages all affine-body-specific external force reporters.
 * It is responsible for:
 * 1. Clearing the external force buffer before constraints update
 * 2. Collecting external forces from all registered reporters
 * 3. Compute the external accelerations after all forces have been applied
 *
 * Hierarchy:
 * - AffineBodyExternalForceManager (this class, derives from ExternalForceReporter)
 *   - Manages AffineBodyExternalForceReporter instances
 *     - AffineBodyExternalBodyForce (applies force to body)
 *     - AffineBodyExternalVertexForce (future: applies force to vertices, sums to body)
 */
class AffineBodyExternalForceManager final : public ExternalForceReporter
{
  public:
    using ExternalForceReporter::ExternalForceReporter;

    static constexpr U64 UID = 666;  // Same UID as external force constitution

    class Impl
    {
      public:
        AffineBodyDynamics* affine_body_dynamics = nullptr;
        SimSystemSlotCollection<AffineBodyExternalForceReporter> m_reporters;

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
        muda::BufferView<Vector12> external_forces() noexcept;

      private:
        friend class AffineBodyExternalForceManager;
        Impl* m_impl = nullptr;
    };

  private:
    virtual void do_build(BuildInfo& info) override;
    virtual U64  get_uid() const noexcept override { return UID; }
    virtual void do_init() override;
    virtual void do_clear() override;  // Clear external force buffers
    virtual void do_step() override;   // Compute accelerations

    friend class AffineBodyExternalForceReporter;
    void register_reporter(AffineBodyExternalForceReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
