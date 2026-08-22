#include <affine_body/abd_al_stiffness_estimator.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_al_stiffness_estimator_estimate_mu_kernel(
        cuda_tool::BufferView<Float>                mu_vertices,
        cuda_tool::CBufferView<IndexT>              body_id,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        Float                                       mu_scale_abd,
        Float                                       dt,
        int                                         n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        mu_vertices(i) = body_masses(body_id(i)).mass() * mu_scale_abd * dt * dt;
    }
}  // namespace

void ABDALStiffnessEstimator::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
    m_impl.vertex_reporter      = require<AffineBodyVertexReporter>();
    m_impl.mu_scale_abd =
        world().scene().config().find<Float>("contact/al-ipc/mu_scale_abd")->view()[0];
}

void ABDALStiffnessEstimator::Impl::estimate_mu(EstimateInfo& info)
{
    UIPC_ASSERT(vertex_reporter->vertex_count() == affine_body_dynamics->v2b().size(),
                "vertex count mismatch");
    int n = (int)vertex_reporter->vertex_count();
    if(n > 0)
    {
        auto k = abd_al_stiffness_estimator_estimate_mu_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.mu_vertices(vertex_reporter->vertex_offset(), vertex_reporter->vertex_count()),
            affine_body_dynamics->v2b().cview(),
            affine_body_dynamics->body_masses().cview(),
            mu_scale_abd,
            info.dt(),
            n);
    }
}

void ABDALStiffnessEstimator::do_estimate_mu(EstimateInfo& info)
{
    m_impl.estimate_mu(info);
}

REGISTER_SIM_SYSTEM(ABDALStiffnessEstimator);

}  // namespace uipc::backend::cuda
