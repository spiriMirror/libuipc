#include <finite_element/fem_al_stiffness_estimator.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FEMALStiffnessEstimator_estimate_mu_kernel(cuda_tool::BufferView<Float> mu_vertices,
                                                               cuda_tool::CBufferView<Float> masses,
                                                               Float mu_scale_fem,
                                                               Float dt,
                                                               int   n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        mu_vertices(idx) = masses(idx) * mu_scale_fem * dt * dt;
    }
}  // namespace

void FEMALStiffnessEstimator::do_build(BuildInfo& info)
{
    m_impl.vertex_reporter       = require<FiniteElementVertexReporter>();
    m_impl.finite_element_method = require<FiniteElementMethod>();
    m_impl.mu_scale_fem =
        world().scene().config().find<Float>("contact/al-ipc/mu_scale_fem")->view()[0];
}

void FEMALStiffnessEstimator::Impl::estimate_mu(EstimateInfo& info)
{
    UIPC_ASSERT(vertex_reporter->vertex_count()
                    == finite_element_method->masses().size(),
                "vertex count mismatch");
    auto k = FEMALStiffnessEstimator_estimate_mu_kernel;
    int  n = (int)vertex_reporter->vertex_count();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.mu_vertices(vertex_reporter->vertex_offset(), vertex_reporter->vertex_count()),
            finite_element_method->masses().cview(),
            mu_scale_fem,
            info.dt(),
            n);
    }
}

void FEMALStiffnessEstimator::do_estimate_mu(EstimateInfo& info)
{
    m_impl.estimate_mu(info);
}

REGISTER_SIM_SYSTEM(FEMALStiffnessEstimator);

}  // namespace uipc::backend::cuda
