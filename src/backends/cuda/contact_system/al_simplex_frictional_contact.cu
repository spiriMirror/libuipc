#include <contact_system/al_contact_function.h>
#include <contact_system/al_simplex_frictional_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
void ALSimplexFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();
    m_impl.global_surf_manager    = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_active_set_manager = require<GlobalActiveSetManager>();

    auto dt_attr = world().scene().config().find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];
}

void ALSimplexFrictionalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.energy_count(0);
        return;
    }

    info.energy_count(active_set->PTs_friction().size()
                      + active_set->EEs_friction().size());
}

void ALSimplexFrictionalContact::do_report_gradient_hessian_extent(
    GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.gradient_count(0);
        info.hessian_count(0);
        return;
    }

    bool gradient_only = info.gradient_only();
    auto pt_count = active_set->PTs_friction().size();
    auto ee_count = active_set->EEs_friction().size();

    SizeT contact_gradient_count = 4 * (pt_count + ee_count);
    SizeT contact_hessian_count =
        pt_count * PTHalfHessianSize + ee_count * EEHalfHessianSize;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(gradient_only ? 0 : contact_hessian_count);
}

void ALSimplexFrictionalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;
    using namespace sym::al_simplex_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PT_size     = active_set->PTs_friction().size();
    auto EE_size     = active_set->EEs_friction().size();
    auto PT_energies = info.energies().subview(0, PT_size);
    auto EE_energies = info.energies().subview(PT_size, EE_size);

    auto x         = global_vertex_manager->positions();
    auto prev_x    = global_vertex_manager->prev_positions();
    auto PTs       = active_set->PTs_friction();
    auto EEs       = active_set->EEs_friction();
    auto PT_lambda = active_set->PT_lambda_friction();
    auto EE_lambda = active_set->EE_lambda_friction();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PT_size,
               [table = global_contact_manager->contact_tabular().cviewer().name("contact_tabular"),
                contact_ids =
                    global_vertex_manager->contact_element_ids().cviewer().name("contact_element_ids"),
                eps_v  = global_contact_manager->eps_velocity(),
                dt     = dt,
                PTs    = PTs.cviewer().name("PTs"),
                lambda = PT_lambda.cviewer().name("lambda"),
                x      = x.cviewer().name("x"),
                prev_x = prev_x.cviewer().name("prev_x"),
                Es = PT_energies.viewer().name("Es")] __device__(int idx) mutable
               {
                   auto     PT   = PTs(idx);
                   Vector4i cids = {contact_ids(PT[0]),
                                    contact_ids(PT[1]),
                                    contact_ids(PT[2]),
                                    contact_ids(PT[3])};

                   auto coeff = sym::codim_ipc_contact::PT_contact_coeff(table, cids);
                   Float mu = coeff.mu;

                   const auto& prev_P  = prev_x(PT[0]);
                   const auto& prev_T0 = prev_x(PT[1]);
                   const auto& prev_T1 = prev_x(PT[2]);
                   const auto& prev_T2 = prev_x(PT[3]);

                   const auto& P  = x(PT[0]);
                   const auto& T0 = x(PT[1]);
                   const auto& T1 = x(PT[2]);
                   const auto& T2 = x(PT[3]);

                   Es(idx) = PT_friction_energy(
                       mu, eps_v * dt, lambda(idx), prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EE_size,
               [table = global_contact_manager->contact_tabular().cviewer().name("contact_tabular"),
                contact_ids =
                    global_vertex_manager->contact_element_ids().cviewer().name("contact_element_ids"),
                eps_v  = global_contact_manager->eps_velocity(),
                dt     = dt,
                EEs    = EEs.cviewer().name("EEs"),
                lambda = EE_lambda.cviewer().name("lambda"),
                x      = x.cviewer().name("x"),
                prev_x = prev_x.cviewer().name("prev_x"),
                Es = EE_energies.viewer().name("Es")] __device__(int idx) mutable
               {
                   auto     EE   = EEs(idx);
                   Vector4i cids = {contact_ids(EE[0]),
                                    contact_ids(EE[1]),
                                    contact_ids(EE[2]),
                                    contact_ids(EE[3])};

                   auto coeff = sym::codim_ipc_contact::EE_contact_coeff(table, cids);
                   Float mu = coeff.mu;

                   const Vector3& prev_Ea0 = prev_x(EE[0]);
                   const Vector3& prev_Ea1 = prev_x(EE[1]);
                   const Vector3& prev_Eb0 = prev_x(EE[2]);
                   const Vector3& prev_Eb1 = prev_x(EE[3]);

                   const Vector3& Ea0 = x(EE[0]);
                   const Vector3& Ea1 = x(EE[1]);
                   const Vector3& Eb0 = x(EE[2]);
                   const Vector3& Eb1 = x(EE[3]);

                   Es(idx) = EE_friction_energy(
                       mu, eps_v * dt, lambda(idx), prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
               });
}

void ALSimplexFrictionalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace muda;
    using namespace sym::al_simplex_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PT_size = active_set->PTs_friction().size();
    auto EE_size = active_set->EEs_friction().size();
    auto PT_grad = info.gradients().subview(0, PT_size * 4);
    auto EE_grad = info.gradients().subview(PT_size * 4, EE_size * 4);
    
    decltype(info.hessians().subview(0, 0)) PT_hess, EE_hess;
    if(info.gradient_only())
    {
        PT_hess = {};
        EE_hess = {};
    }
    else
    {
        PT_hess = info.hessians().subview(0, PT_size * PTHalfHessianSize);
        EE_hess = info.hessians().subview(PT_size * PTHalfHessianSize, EE_size * EEHalfHessianSize);
    }

    auto x         = global_vertex_manager->positions();
    auto prev_x    = global_vertex_manager->prev_positions();
    auto rest_x    = global_vertex_manager->rest_positions();
    auto PTs       = active_set->PTs_friction();
    auto EEs       = active_set->EEs_friction();
    auto PT_lambda = active_set->PT_lambda_friction();
    auto EE_lambda = active_set->EE_lambda_friction();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PT_size,
               [table = global_contact_manager->contact_tabular().cviewer().name("contact_tabular"),
                contact_ids =
                    global_vertex_manager->contact_element_ids().cviewer().name("contact_element_ids"),
                eps_v  = global_contact_manager->eps_velocity(),
                dt     = dt,
                PTs    = PTs.cviewer().name("PTs"),
                lambda = PT_lambda.cviewer().name("lambda"),
                x      = x.cviewer().name("x"),
                prev_x = prev_x.cviewer().name("prev_x"),
                gradient_only = info.gradient_only(),
                Gs     = PT_grad.viewer().name("Gs"),
                Hs = PT_hess.viewer().name("Hs")] __device__(int idx) mutable
               {
                   auto     PT   = PTs(idx);
                   Vector4i cids = {contact_ids(PT[0]),
                                    contact_ids(PT[1]),
                                    contact_ids(PT[2]),
                                    contact_ids(PT[3])};

                   auto coeff = sym::codim_ipc_contact::PT_contact_coeff(table, cids);
                   Float mu = coeff.mu;

                   const auto& prev_P  = prev_x(PT[0]);
                   const auto& prev_T0 = prev_x(PT[1]);
                   const auto& prev_T1 = prev_x(PT[2]);
                   const auto& prev_T2 = prev_x(PT[3]);

                   const auto& P  = x(PT[0]);
                   const auto& T0 = x(PT[1]);
                   const auto& T1 = x(PT[2]);
                   const auto& T2 = x(PT[3]);

                   Vector12    G;
                   Matrix12x12 H;

                   PT_friction_gradient_hessian(
                       G, H, mu, eps_v * dt, lambda(idx), prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);

                   DoubletVectorAssembler DVA{Gs};
                   DVA.segment<4>(idx * 4).write(PT, G);

                   if(!gradient_only)
                   {
                       TripletMatrixAssembler TMA{Hs};
                       TMA.half_block<4>(idx * PTHalfHessianSize).write(PT, H);
                   }
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EE_size,
               [table = global_contact_manager->contact_tabular().cviewer().name("contact_tabular"),
                contact_ids =
                    global_vertex_manager->contact_element_ids().cviewer().name("contact_element_ids"),
                eps_v  = global_contact_manager->eps_velocity(),
                dt     = dt,
                EEs    = EEs.cviewer().name("EEs"),
                lambda = EE_lambda.cviewer().name("lambda"),
                x      = x.cviewer().name("x"),
                rest_x = rest_x.viewer().name("rest_x"),
                prev_x = prev_x.cviewer().name("prev_x"),
                gradient_only = info.gradient_only(),
                Gs     = EE_grad.viewer().name("Gs"),
                Hs = EE_hess.viewer().name("Hs")] __device__(int idx) mutable
               {
                   auto     EE   = EEs(idx);
                   Vector4i cids = {contact_ids(EE[0]),
                                    contact_ids(EE[1]),
                                    contact_ids(EE[2]),
                                    contact_ids(EE[3])};

                   auto coeff = sym::codim_ipc_contact::EE_contact_coeff(table, cids);
                   Float mu = coeff.mu;

                   const Vector3& rest_Ea0 = rest_x(EE[0]);
                   const Vector3& rest_Ea1 = rest_x(EE[1]);
                   const Vector3& rest_Eb0 = rest_x(EE[2]);
                   const Vector3& rest_Eb1 = rest_x(EE[3]);

                   const Vector3& prev_Ea0 = prev_x(EE[0]);
                   const Vector3& prev_Ea1 = prev_x(EE[1]);
                   const Vector3& prev_Eb0 = prev_x(EE[2]);
                   const Vector3& prev_Eb1 = prev_x(EE[3]);

                   const Vector3& Ea0 = x(EE[0]);
                   const Vector3& Ea1 = x(EE[1]);
                   const Vector3& Eb0 = x(EE[2]);
                   const Vector3& Eb1 = x(EE[3]);

                   Float eps_x;
                   distance::edge_edge_mollifier_threshold(
                       rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, eps_x);
                   if(!distance::need_mollify(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, eps_x))
                   {
                       Vector12    G;
                       Matrix12x12 H;

                       EE_friction_gradient_hessian(
                           G, H, mu, eps_v * dt, lambda(idx), prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);

                       DoubletVectorAssembler DVA{Gs};
                       DVA.segment<4>(idx * 4).write(EE, G);

                       if(!gradient_only)
                       {
                           TripletMatrixAssembler TMA{Hs};
                           TMA.half_block<4>(idx * EEHalfHessianSize).write(EE, H);
                       }
                   }
               });
}

void ALSimplexFrictionalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    m_impl.do_compute_energy(info);
}

void ALSimplexFrictionalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    m_impl.do_assemble(info);
}

REGISTER_SIM_SYSTEM(ALSimplexFrictionalContact);
}  // namespace uipc::backend::cuda
