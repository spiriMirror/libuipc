#include <contact_system/al_vertex_half_plane_frictional_contact.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <contact_system/al_vertex_half_plane_contact_function.h>
#include <pipeline/al_ipc_pipeline_flag.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void do_compute_energy_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                             cuda_tool::CBufferView<IndexT> contact_ids,
                                             IndexT half_plane_vertex_offset,
                                             Float  eps_v,
                                             Float  dt,
                                             cuda_tool::CBufferView<Vector2i> PHs,
                                             cuda_tool::CBufferView<Float> lambda,
                                             cuda_tool::CBufferView<Vector3> x,
                                             cuda_tool::CBufferView<Vector3> prev_x,
                                             cuda_tool::CBufferView<Vector3> plane_normals,
                                             cuda_tool::BufferView<Float> Es,
                                             int                          n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_vertex_half_plane_contact;

        auto vI = PHs(idx)(0), HI = PHs(idx)(1);

        ContactCoeff coeff =
            table(contact_ids(vI), contact_ids(HI + half_plane_vertex_offset));
        Float mu = coeff.mu;

        const auto& N            = plane_normals(HI);
        auto        normal_force = lambda(idx);

        Es(idx) = half_plane_frictional_energy(
            mu, eps_v * dt, normal_force, x(vI), prev_x(vI), N);
    }

    __global__ void do_assemble_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                       cuda_tool::CBufferView<IndexT> contact_ids,
                                       IndexT half_plane_vertex_offset,
                                       Float  eps_v,
                                       Float  dt,
                                       cuda_tool::CBufferView<Vector2i> PHs,
                                       cuda_tool::CBufferView<Float>    lambda,
                                       cuda_tool::CBufferView<Vector3>  x,
                                       cuda_tool::CBufferView<Vector3>  prev_x,
                                       cuda_tool::CBufferView<Vector3> plane_normals,
                                       cuda_tool::DoubletVectorView<Float, 3> Gs,
                                       cuda_tool::TripletMatrixView<Float, 3> Hs,
                                       int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_vertex_half_plane_contact;

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
    }
}  // namespace

void ALVertexHalfPlaneFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    require<ALIPCPipelineFlag>();
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();
    m_impl.global_surf_manager    = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_active_set_manager  = require<GlobalActiveSetManager>();
    m_impl.half_plane_vertex_reporter = require<HalfPlaneVertexReporter>();
    m_impl.half_plane                 = require<HalfPlane>();

    m_impl.dt_attr = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");
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
    using namespace cuda_tool;
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

    if(PH_size > 0)
        do_compute_energy_kernel<<<cuda_tool::best_grid_dim((int)PH_size, do_compute_energy_kernel), cuda_tool::best_block_dim(do_compute_energy_kernel), 0, nullptr>>>(
            global_contact_manager->contact_tabular().cviewer(),
            global_vertex_manager->contact_element_ids().cviewer(),
            half_plane_vertex_reporter->vertex_offset(),
            global_contact_manager->eps_velocity(),
            dt_attr->view()[0],
            PHs.cviewer(),
            PH_lambda.cviewer(),
            x.cviewer(),
            prev_x.cviewer(),
            half_plane->normals().viewer(),
            PH_energies.viewer(),
            (int)PH_size);
}

void ALVertexHalfPlaneFrictionalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace cuda_tool;
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

    if(PH_size > 0)
        do_assemble_kernel<<<cuda_tool::best_grid_dim((int)PH_size, do_assemble_kernel), cuda_tool::best_block_dim(do_assemble_kernel), 0, nullptr>>>(
            global_contact_manager->contact_tabular().cviewer(),
            global_vertex_manager->contact_element_ids().cviewer(),
            half_plane_vertex_reporter->vertex_offset(),
            global_contact_manager->eps_velocity(),
            dt_attr->view()[0],
            PHs.cviewer(),
            PH_lambda.cviewer(),
            x.cviewer(),
            prev_x.cviewer(),
            half_plane->normals().viewer(),
            PH_grad.viewer(),
            PH_hess.viewer(),
            (int)PH_size);
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
