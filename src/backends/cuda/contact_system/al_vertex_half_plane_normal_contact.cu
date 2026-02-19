#include <contact_system/al_vertex_half_plane_normal_contact.h>
#include <contact_system/al_vertex_half_plane_contact_function.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
void ALVertexHalfPlaneNormalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();
    m_impl.global_surf_manager    = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_active_set_manager = require<GlobalActiveSetManager>();
}

void ALVertexHalfPlaneNormalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.energy_count(0);
        return;
    }

    info.energy_count(active_set->PHs().size());
}

void ALVertexHalfPlaneNormalContact::do_report_gradient_hessian_extent(
    GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.gradient_count(0);
        info.hessian_count(0);
        return;
    }

    SizeT count = active_set->PHs().size();
    info.gradient_count(count);

    if(info.gradient_only())
        return;

    info.hessian_count(count);
}

void ALVertexHalfPlaneNormalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;
    using namespace sym::al_vertex_half_plane_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PH_size     = active_set->PHs().size();
    auto PH_energies = info.energies().subview(0, PH_size);

    auto x         = global_vertex_manager->positions();
    auto PHs       = active_set->PHs();
    auto PH_d0     = active_set->PH_d0();
    auto PH_cnt    = active_set->PH_cnt();
    auto PH_d_grad = active_set->PH_d_grad();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PH_size,
               [mu     = active_set->mu(),
                decay  = active_set->decay_factor(),
                PHs    = PHs.cviewer().name("PHs"),
                cnt    = PH_cnt.cviewer().name("cnt"),
                d0     = PH_d0.cviewer().name("d0"),
                d_grad = PH_d_grad.cviewer().name("d_grad"),
                x      = x.cviewer().name("x"),
                Es = PH_energies.viewer().name("Es")] __device__(int idx) mutable
               {
                   auto vI = PHs(idx);
                   Es(idx) = half_plane_penalty_energy(
                       pow(decay, cnt(idx)) * mu, d0(idx), d_grad(idx), x(vI));
               });
}

void ALVertexHalfPlaneNormalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace muda;
    using namespace sym::al_vertex_half_plane_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PH_size = active_set->PHs().size();
    auto PH_grad = info.gradients().subview(0, PH_size);
    auto PH_hess = info.hessians().subview(0, PH_size);

    auto x         = global_vertex_manager->positions();
    auto PHs       = active_set->PHs();
    auto PH_d0     = active_set->PH_d0();
    auto PH_cnt    = active_set->PH_cnt();
    auto PH_d_grad = active_set->PH_d_grad();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PH_size,
               [mu          = active_set->mu(),
                decay       = active_set->decay_factor(),
                PHs         = PHs.cviewer().name("PHs"),
                cnt         = PH_cnt.cviewer().name("cnt"),
                d0          = PH_d0.cviewer().name("d0"),
                d_grad      = PH_d_grad.cviewer().name("d_grad"),
                x           = x.cviewer().name("x"),
                gradient_only = info.gradient_only(),
                Gs          = PH_grad.viewer().name("Gs"),
                Hs          = PH_hess.viewer().name("Hs")] __device__(int idx) mutable
               {
                   auto      vI = PHs(idx);
                   Vector3   G;
                   Matrix3x3 H;
                   half_plane_penalty_gradient_hessian(
                       pow(decay, cnt(idx)) * mu, d0(idx), d_grad(idx), x(vI), G, H);

                   Gs(idx).write(vI, G);
                   if(!gradient_only)
                   {
                       Hs(idx).write(vI, vI, H);
                   }
               });
}

void ALVertexHalfPlaneNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    m_impl.do_compute_energy(info);
}

void ALVertexHalfPlaneNormalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    m_impl.do_assemble(info);
}

REGISTER_SIM_SYSTEM(ALVertexHalfPlaneNormalContact);
}  // namespace uipc::backend::cuda
