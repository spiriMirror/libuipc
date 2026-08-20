#include <contact_system/al_simplex_normal_contact.h>
#include <contact_system/al_contact_function.h>
#include <pipeline/al_ipc_pipeline_flag.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/log.h>

namespace uipc::backend::cuda
{
void ALSimplexNormalContact::do_build(ContactReporter::BuildInfo& info)
{
    require<ALIPCPipelineFlag>();
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();
    m_impl.global_surf_manager    = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_active_set_manager = require<GlobalActiveSetManager>();
}

void ALSimplexNormalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.energy_count(0);
        return;
    }

    info.energy_count(active_set->PTs().size() + active_set->EEs().size());
}

void ALSimplexNormalContact::do_report_gradient_hessian_extent(GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.gradient_count(0);
        info.hessian_count(0);
        return;
    }

    info.gradient_count(4 * (active_set->PTs().size() + active_set->EEs().size()));
    info.hessian_count(10 * (active_set->PTs().size() + active_set->EEs().size()));
}

void ALSimplexNormalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace cuda_tool;
    using namespace sym::al_simplex_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PT_size = active_set->PTs().size(), EE_size = active_set->EEs().size();
    auto PT_energies = info.energies().subview(0, PT_size);
    auto EE_energies = info.energies().subview(PT_size, EE_size);

    auto x   = global_vertex_manager->positions();
    auto PTs = active_set->PTs(), EEs = active_set->EEs();
    auto PT_d0 = active_set->PT_d0(), EE_d0 = active_set->EE_d0();
    auto PT_cnt = active_set->PT_cnt(), EE_cnt = active_set->EE_cnt();
    auto PT_d_grad = active_set->PT_d_grad();
    auto EE_d_grad = active_set->EE_d_grad();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PT_size,
               [mu_v   = active_set->mu_vertices().cviewer(),
                decay  = active_set->decay_factor(),
                PTs    = PTs.cviewer(),
                cnt    = PT_cnt.cviewer(),
                d0     = PT_d0.cviewer(),
                d_grad = PT_d_grad.cviewer(),
                x      = x.cviewer(),
                Es = PT_energies.viewer()] __device__(int idx) mutable
               {
                   auto PT = PTs(idx);
                   auto mu = min(min(mu_v(PT(0)), mu_v(PT(1))),
                                 min(mu_v(PT(2)), mu_v(PT(3))));
                   auto c  = cnt(idx) >= 0 ? cnt(idx) : max(-cnt(idx) - 6, 0);
                   Es(idx) = penalty_energy(pow(decay, c) * mu,
                                            d0(idx),
                                            d_grad(idx),
                                            x(PT(0)),
                                            x(PT(1)),
                                            x(PT(2)),
                                            x(PT(3)));
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EE_size,
               [mu_v   = active_set->mu_vertices().cviewer(),
                decay  = active_set->decay_factor(),
                EEs    = EEs.cviewer(),
                cnt    = EE_cnt.cviewer(),
                d0     = EE_d0.cviewer(),
                d_grad = EE_d_grad.cviewer(),
                x      = x.cviewer(),
                Es = EE_energies.viewer()] __device__(int idx) mutable
               {
                   auto EE = EEs(idx);
                   auto mu = min(min(mu_v(EE(0)), mu_v(EE(1))),
                                 min(mu_v(EE(2)), mu_v(EE(3))));
                   auto c  = cnt(idx) >= 0 ? cnt(idx) : max(-cnt(idx) - 6, 0);
                   Es(idx) = penalty_energy(pow(decay, c) * mu,
                                            d0(idx),
                                            d_grad(idx),
                                            x(EE(0)),
                                            x(EE(1)),
                                            x(EE(2)),
                                            x(EE(3)));
               });
}

void ALSimplexNormalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace cuda_tool;
    using namespace sym::al_simplex_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PT_size = active_set->PTs().size(), EE_size = active_set->EEs().size();
    auto PT_grad = info.gradients().subview(0, PT_size * 4);
    auto EE_grad = info.gradients().subview(PT_size * 4, EE_size * 4);
    auto PT_hess = info.hessians().subview(0, PT_size * 10);
    auto EE_hess = info.hessians().subview(PT_size * 10, EE_size * 10);

    auto x   = global_vertex_manager->positions();
    auto PTs = active_set->PTs(), EEs = active_set->EEs();
    auto PT_d0 = active_set->PT_d0(), EE_d0 = active_set->EE_d0();
    auto PT_cnt = active_set->PT_cnt(), EE_cnt = active_set->EE_cnt();
    auto PT_d_grad = active_set->PT_d_grad();
    auto EE_d_grad = active_set->EE_d_grad();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PT_size,
               [mu_v   = active_set->mu_vertices().cviewer(),
                decay  = active_set->decay_factor(),
                PTs    = PTs.cviewer(),
                cnt    = PT_cnt.cviewer(),
                d0     = PT_d0.cviewer(),
                d_grad = PT_d_grad.cviewer(),
                x      = x.cviewer(),
                Gs     = PT_grad.viewer(),
                Hs = PT_hess.viewer()] __device__(int idx) mutable
               {
                   auto        PT = PTs(idx);
                   auto        mu = min(min(mu_v(PT(0)), mu_v(PT(1))),
                                 min(mu_v(PT(2)), mu_v(PT(3))));
                   Vector12    G;
                   Matrix12x12 H;
                   auto c = cnt(idx) >= 0 ? cnt(idx) : max(-cnt(idx) - 6, 0);
                   penalty_gradient_hessian(pow(decay, c) * mu,
                                            d0(idx),
                                            d_grad(idx),
                                            x(PT(0)),
                                            x(PT(1)),
                                            x(PT(2)),
                                            x(PT(3)),
                                            G,
                                            H);

                   DoubletVectorAssembler DVA{Gs};
                   DVA.segment<4>(idx * 4).write(PT, G);

                   TripletMatrixAssembler TMA{Hs};
                   TMA.half_block<4>(idx * 10).write(PT, H);
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EE_size,
               [mu_v   = active_set->mu_vertices().cviewer(),
                decay  = active_set->decay_factor(),
                EEs    = EEs.cviewer(),
                cnt    = EE_cnt.cviewer(),
                d0     = EE_d0.cviewer(),
                d_grad = EE_d_grad.cviewer(),
                x      = x.cviewer(),
                Gs     = EE_grad.viewer(),
                Hs = EE_hess.viewer()] __device__(int idx) mutable
               {
                   auto        EE = EEs(idx);
                   auto        mu = min(min(mu_v(EE(0)), mu_v(EE(1))),
                                 min(mu_v(EE(2)), mu_v(EE(3))));
                   Vector12    G;
                   Matrix12x12 H;
                   auto c = cnt(idx) >= 0 ? cnt(idx) : max(-cnt(idx) - 6, 0);
                   penalty_gradient_hessian(pow(decay, c) * mu,
                                            d0(idx),
                                            d_grad(idx),
                                            x(EE(0)),
                                            x(EE(1)),
                                            x(EE(2)),
                                            x(EE(3)),
                                            G,
                                            H);

                   DoubletVectorAssembler DVA{Gs};
                   DVA.segment<4>(idx * 4).write(EE, G);

                   TripletMatrixAssembler TMA{Hs};
                   TMA.half_block<4>(idx * 10).write(EE, H);
               });
}

void ALSimplexNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    m_impl.do_compute_energy(info);
}

void ALSimplexNormalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    m_impl.do_assemble(info);
}

REGISTER_SIM_SYSTEM(ALSimplexNormalContact);
}  // namespace uipc::backend::cuda
