#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/affine_body_constitution.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDLineSearchReporter);

void ABDLineSearchReporter::do_build(LineSearchReporter::BuildInfo& info)
{

    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
    auto aba                    = find<AffineBodyAnimator>();
    if(aba)
        m_impl.affine_body_animator = *aba;
}

void ABDLineSearchReporter::do_record_start_point(LineSearcher::RecordInfo& info)
{
    m_impl.record_start_point(info);
}

void ABDLineSearchReporter::do_step_forward(LineSearcher::StepInfo& info)
{
    m_impl.step_forward(info);
}

void ABDLineSearchReporter::do_compute_energy(LineSearcher::EnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void ABDLineSearchReporter::Impl::record_start_point(LineSearcher::RecordInfo& info)
{
    using namespace muda;

    BufferLaunch().template copy<Vector12>(abd().body_id_to_q_temp.view(),
                                           abd().body_id_to_q.view());
}

void ABDLineSearchReporter::Impl::step_forward(LineSearcher::StepInfo& info)
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(abd().abd_body_count,
               [is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                q_temps  = abd().body_id_to_q_temp.cviewer().name("q_temps"),
                qs       = abd().body_id_to_q.viewer().name("qs"),
                dqs      = abd().body_id_to_dq.cviewer().name("dqs"),
                alpha    = info.alpha] __device__(int i) mutable
               {
                   if(is_fixed(i))
                       return;
                   qs(i) = q_temps(i) + alpha * dqs(i);
               });
}

void ABDLineSearchReporter::Impl::compute_energy(LineSearcher::EnergyInfo& info)
{
    using namespace muda;

    // Compute kinetic energy
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(abd().abd_body_count,
               [is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                qs       = abd().body_id_to_q.cviewer().name("qs"),
                q_tildes = abd().body_id_to_q_tilde.viewer().name("q_tildes"),
                q_prevs  = abd().body_id_to_q_prev.viewer().name("q_prevs"),
                gravities = abd().body_id_to_abd_gravity.cviewer().name("gravities"),
                masses = abd().body_id_to_abd_mass.cviewer().name("masses"),
                Ks     = abd().body_id_to_kinetic_energy.viewer().name(
                    "kinetic_energy")] __device__(int i) mutable
               {
                   auto& K = Ks(i);
                   if(is_fixed(i))
                   {
                       K = 0.0;
                   }
                   else
                   {
                       const auto& q       = qs(i);
                       const auto& q_tilde = q_tildes(i);
                       const auto& M       = masses(i);
                       Vector12    dq      = q - q_tilde;
                       K                   = 0.5 * dq.dot(M * dq);
                   }
               });

    DeviceReduce().Sum(abd().body_id_to_kinetic_energy.data(),
                       abd().abd_kinetic_energy.data(),
                       abd().body_id_to_kinetic_energy.size());

    Float K = abd().abd_kinetic_energy;

    // distribute the computation of shape energy to each constitution
    for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
    {
        AffineBodyDynamics::ComputeEnergyInfo this_info{&abd(), i, info.dt()};
        cst->compute_energy(this_info);
    }

    // sum up the shape energy
    DeviceReduce().Sum(abd().body_id_to_shape_energy.data(),
                       abd().abd_shape_energy.data(),
                       abd().body_id_to_shape_energy.size());

    Float shape_E = abd().abd_shape_energy;

    Float anim_E = 0;
    if(affine_body_animator)
        anim_E = affine_body_animator->compute_energy(info);

    Float E = K + shape_E + anim_E;

    // spdlog::info("ABD Energy: K: {}, Shape: {}, Anim: {}", K, shape_E, anim_E);

    info.energy(E);
}
}  // namespace uipc::backend::cuda
