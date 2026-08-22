#include <finite_element/finite_element_external_force_reporter.h>
#include <finite_element/constraints/finite_element_external_vertex_force_constraint.h>
#include <finite_element/finite_element_method.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace eigen = cuda_tool::eigen;

    __global__ void FiniteElementExternalVertexForce_do_step_kernel(
        cuda_tool::BufferView<Vector3>  forces,
        cuda_tool::CBufferView<IndexT>  vertex_ids,
        cuda_tool::CBufferView<Vector3> vertex_forces,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto vid = vertex_ids(i);
        eigen::atomic_add(forces(vid), vertex_forces(i));
    }
}  // namespace

/**
 * @brief Scatter-add external forces from constraint buffers into the
 *        per-vertex external force buffer on FiniteElementMethod.
 */
class FiniteElementExternalVertexForce final : public FiniteElementExternalForceReporter
{
  public:
    static constexpr U64 UID = 671;

    using FiniteElementExternalForceReporter::FiniteElementExternalForceReporter;

    SimSystemSlot<FiniteElementMethod> finite_element_method;
    SimSystemSlot<FiniteElementExternalVertexForceConstraint> constraint;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method = require<FiniteElementMethod>();
        constraint = require<FiniteElementExternalVertexForceConstraint>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init() override {}

    void do_step(ExternalForceInfo& info) override
    {
        auto k = FiniteElementExternalVertexForce_do_step_kernel;
        int  n = (int)constraint->forces().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.external_forces(), constraint->vertex_ids(), constraint->forces(), n);
        }
    }
};

REGISTER_SIM_SYSTEM(FiniteElementExternalVertexForce);
}  // namespace uipc::backend::cuda
