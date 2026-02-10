#include <contact_system/al_vertex_half_plane_frictional_contact.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <contact_system/al_vertex_half_plane_contact_function.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
void ALVertexHalfPlaneFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();
    m_impl.global_surf_manager    = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_active_set_manager  = require<GlobalActiveSetManager>();
    m_impl.half_plane_vertex_reporter = require<HalfPlaneVertexReporter>();
    m_impl.half_plane                 = require<HalfPlane>();

    auto dt_attr = world().scene().config().find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];
}

void ALVertexHalfPlaneFrictionalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.energy_count(0);
        return;
    }

    info.energy_count(active_set->PHs_friction().size());
}

void ALVertexHalfPlaneFrictionalContact::do_report_gradient_hessian_extent(
    GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& active_set = m_impl.global_active_set_manager;

    if(!active_set->is_enabled())
    {
        info.gradient_count(0);
        info.hessian_count(0);
        return;
    }

    info.gradient_count(active_set->PHs_friction().size());
    info.hessian_count(active_set->PHs_friction().size());
}

void ALVertexHalfPlaneFrictionalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;
    using namespace sym::al_vertex_half_plane_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PH_size     = active_set->PHs_friction().size();
    auto PH_energies = info.energies().subview(0, PH_size);

    auto x         = global_vertex_manager->positions();
    auto prev_x    = global_vertex_manager->prev_positions();
    auto PHs       = active_set->PHs_friction();
    auto PH_lambda = active_set->PH_lambda_friction();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PH_size,
               [table = global_contact_manager->contact_tabular().cviewer().name("contact_tabular"),
                contact_ids =
                    global_vertex_manager->contact_element_ids().cviewer().name("contact_element_ids"),
                half_plane_vertex_offset = half_plane_vertex_reporter->vertex_offset(),
                eps_v  = global_contact_manager->eps_velocity(),
                dt     = dt,
                PHs    = PHs.cviewer().name("PHs"),
                lambda = PH_lambda.cviewer().name("lambda"),
                x      = x.cviewer().name("x"),
                prev_x = prev_x.cviewer().name("prev_x"),
                plane_positions = half_plane->positions().viewer().name("plane_positions"),
                plane_normals = half_plane->normals().viewer().name("plane_normals"),
                Es = PH_energies.viewer().name("Es")] __device__(int idx) mutable
               {
                   auto vI = PHs(idx)(0), HI = PHs(idx)(1);

                   ContactCoeff coeff =
                       table(contact_ids(vI), contact_ids(HI + half_plane_vertex_offset));
                   Float mu = coeff.mu;

                   const auto& N            = plane_normals(HI);
                   auto        normal_force = lambda(idx);

                   Es(idx) = half_plane_frictional_energy(
                       mu, eps_v * dt, normal_force, x(vI), prev_x(vI), N);
               });
}

void ALVertexHalfPlaneFrictionalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace muda;
    using namespace sym::al_vertex_half_plane_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PH_size = active_set->PHs_friction().size();
    auto PH_grad = info.gradients().subview(0, PH_size);
    auto PH_hess = info.hessians().subview(0, PH_size);

    auto x         = global_vertex_manager->positions();
    auto prev_x    = global_vertex_manager->prev_positions();
    auto PHs       = active_set->PHs_friction();
    auto PH_lambda = active_set->PH_lambda_friction();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PH_size,
               [table = global_contact_manager->contact_tabular().cviewer().name("contact_tabular"),
                contact_ids =
                    global_vertex_manager->contact_element_ids().cviewer().name("contact_element_ids"),
                half_plane_vertex_offset = half_plane_vertex_reporter->vertex_offset(),
                eps_v  = global_contact_manager->eps_velocity(),
                dt     = dt,
                PHs    = PHs.cviewer().name("PHs"),
                lambda = PH_lambda.cviewer().name("lambda"),
                x      = x.cviewer().name("x"),
                prev_x = prev_x.cviewer().name("prev_x"),
                plane_positions = half_plane->positions().viewer().name("plane_positions"),
                plane_normals = half_plane->normals().viewer().name("plane_normals"),
                Gs = PH_grad.viewer().name("Gs"),
                Hs = PH_hess.viewer().name("Hs")] __device__(int idx) mutable
               {
                   auto vI = PHs(idx)(0), HI = PHs(idx)(1);

                   ContactCoeff coeff =
                       table(contact_ids(vI), contact_ids(HI + half_plane_vertex_offset));
                   Float mu = coeff.mu;

                   const auto& N            = plane_normals(HI);
                   auto        normal_force = lambda(idx);

                   Vector3   G;
                   Matrix3x3 H;
                   half_plane_frictional_gradient_hessian(
                       G, H, mu, eps_v * dt, normal_force, x(vI), prev_x(vI), N);

                   Gs(idx).write(vI, G);
                   Hs(idx).write(vI, vI, H);
               });
}

void ALVertexHalfPlaneFrictionalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    m_impl.do_compute_energy(info);
}

void ALVertexHalfPlaneFrictionalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    m_impl.do_assemble(info);
}

REGISTER_SIM_SYSTEM(ALVertexHalfPlaneFrictionalContact);
}  // namespace uipc::backend::cuda
