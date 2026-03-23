#include <affine_body/abd_al_stiffness_estimator.h>

namespace uipc::backend::cuda
{

void ABDALStiffnessEstimator::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
    m_impl.vertex_reporter      = require<AffineBodyVertexReporter>();
    m_impl.mu_scale_abd =
        world().scene().config().find<Float>("contact/al-ipc/mu_scale_abd")->view()[0];
}

void ABDALStiffnessEstimator::Impl::estimate_mu(EstimateInfo& info)
{
    using namespace muda;
    UIPC_ASSERT(vertex_reporter->vertex_count() == affine_body_dynamics->v2b().size(),
                "vertex count mismatch");
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(vertex_reporter->vertex_count(),
               [mu_vertices = info.mu_vertices(vertex_reporter->vertex_offset(),
                                               vertex_reporter->vertex_count())
                                  .viewer()
                                  .name("mu_vertices"),
                body_id = affine_body_dynamics->v2b().cviewer().name("body_id"),
                body_masses = affine_body_dynamics->body_masses().cviewer().name("body_masses"),
                mu_scale_abd = mu_scale_abd,
                dt           = info.dt()] __device__(int idx) mutable
               {
                   mu_vertices(idx) =
                       body_masses(body_id(idx)).mass() * mu_scale_abd * dt * dt;
               });
}

void ABDALStiffnessEstimator::do_estimate_mu(EstimateInfo& info)
{
    m_impl.estimate_mu(info);
}

REGISTER_SIM_SYSTEM(ABDALStiffnessEstimator);

}  // namespace uipc::backend::cuda
