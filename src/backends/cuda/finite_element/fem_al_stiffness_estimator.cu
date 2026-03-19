#include <finite_element/fem_al_stiffness_estimator.h>

namespace uipc::backend::cuda
{

void FEMALStiffnessEstimator::do_build(BuildInfo& info)
{
    m_impl.vertex_reporter       = require<FiniteElementVertexReporter>();
    m_impl.finite_element_method = require<FiniteElementMethod>();
    m_impl.mu_scale_fem =
        world().scene().config().find<Float>("contact/al-ipc/mu_scale_fem")->view()[0];
}

void FEMALStiffnessEstimator::Impl::estimate_mu(EstimateInfo& info)
{
    using namespace muda;
    UIPC_ASSERT(vertex_reporter->vertex_count()
                    == finite_element_method->masses().size(),
                "vertex count mismatch");
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(vertex_reporter->vertex_count(),
               [mu_vertices = info.mu_vertices(vertex_reporter->vertex_offset(),
                                               vertex_reporter->vertex_count())
                                  .viewer()
                                  .name("mu_vertices"),
                masses = finite_element_method->masses().cviewer().name("masses"),
                mu_scale_fem = mu_scale_fem,
                dt           = info.dt()] __device__(int idx) mutable
               { mu_vertices(idx) = masses(idx) * mu_scale_fem * dt * dt; });
}

void FEMALStiffnessEstimator::do_estimate_mu(EstimateInfo& info)
{
    m_impl.estimate_mu(info);
}

REGISTER_SIM_SYSTEM(FEMALStiffnessEstimator);

}  // namespace uipc::backend::cuda
