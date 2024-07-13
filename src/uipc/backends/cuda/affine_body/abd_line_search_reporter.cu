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
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
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
    ParallelFor(256)
        .kernel_name(__FUNCTION__)
        .apply(abd().body_count(),
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
        .kernel_name(__FUNCTION__)
        .apply(abd().body_count(),
               [is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                qs       = abd().body_id_to_q.cviewer().name("qs"),
                q_tildes = abd().body_id_to_q_tilde.viewer().name("q_tildes"),
                masses   = abd().body_id_to_abd_mass.cviewer().name("masses"),
                Ks       = abd().body_id_to_kinetic_energy.viewer().name(
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
                       K = 0.5 * dq.dot(M * dq);
                   }
               });

    DeviceReduce().Sum(abd().body_id_to_kinetic_energy.data(),
                       abd().abd_kinetic_energy.data(),
                       abd().body_id_to_kinetic_energy.size());

    Float K = abd().abd_kinetic_energy;


    // Compute shape energy
    auto async_fill = []<typename T>(muda::DeviceBuffer<T>& buf, const T& value)
    { muda::BufferLaunch().fill<T>(buf.view(), value); };

    async_fill(abd().constitution_shape_energy, 0.0);

    // _distribute the computation of shape energy to each constitution
    for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
    {
        auto body_offset = abd().h_constitution_body_offsets[i];
        auto body_count  = abd().h_constitution_body_counts[i];

        if(body_count == 0)
            continue;

        AffineBodyDynamics::ComputeEnergyInfo this_info{
            &abd(), i, VarView<Float>{abd().constitution_shape_energy.data() + i}, info.dt()};
        cst->compute_energy(this_info);
    }
    abd().constitution_shape_energy.view().copy_to(
        abd().h_constitution_shape_energy.data());

    // sum up the shape energy
    Float E = std::accumulate(abd().h_constitution_shape_energy.begin(),
                              abd().h_constitution_shape_energy.end(),
                              0.0);

    // spdlog::info("ABD K: {}, E: {}", K, E);

    info.energy(K + E);
}
}  // namespace uipc::backend::cuda
