#include <affine_body/affine_body_external_force_reporter.h>
#include <affine_body/constraints/affine_body_external_body_force_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
namespace
{
    using namespace cuda_tool;

    __global__ void affine_body_external_body_force_do_step_kernel(
        cuda_tool::BufferView<Vector12>  forces,
        cuda_tool::CBufferView<IndexT>   body_ids,
        cuda_tool::CBufferView<Vector12> body_forces,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // Scatter add the external forces to the corresponding bodies
        auto body_id = body_ids(i);
        eigen::atomic_add(forces(body_id), body_forces(i));
    }
}  // namespace

/**
 * @brief Get external forces from ExternalForceConstraint and apply them to Affine Bodies
 *
 * This reporter add forces to Affine Bodies in the AffineBodyDynamics system.
 *
 * This is the "body force" implementation - forces are applied directly to bodies.
 * Future implementations like AffineBodyExternalVertexForce may apply forces to vertices.
 */
class AffineBodyExternalBodyForce final : public AffineBodyExternalForceReporter
{
  public:
    static constexpr U64 UID = 666;  // Same UID as ExternalForceConstraint

    using AffineBodyExternalForceReporter::AffineBodyExternalForceReporter;

    SimSystemSlot<AffineBodyDynamics>                    affine_body_dynamics;
    SimSystemSlot<AffineBodyExternalBodyForceConstraint> constraint;

    virtual void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        constraint           = require<AffineBodyExternalBodyForceConstraint>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override
    {
        // Nothing to do
    }

    void do_step(ExternalForceInfo& info) override
    {
        SizeT force_count = constraint->forces().size();

        int n = (int)force_count;
        if(n > 0)
        {
            auto k = affine_body_external_body_force_do_step_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.external_forces(), constraint->body_ids(), constraint->forces(), n);
        }
    }
};

REGISTER_SIM_SYSTEM(AffineBodyExternalBodyForce);
}  // namespace uipc::backend::cuda
