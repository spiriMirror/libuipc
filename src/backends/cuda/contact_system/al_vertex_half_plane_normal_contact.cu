#include <contact_system/al_vertex_half_plane_normal_contact.h>
#include <contact_system/al_vertex_half_plane_contact_function.h>
#include <pipeline/al_ipc_pipeline_flag.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void do_compute_energy_kernel(cuda_tool::CBufferView<Float> mu_v,
                                             Float                       decay,
                                             cuda_tool::CBufferView<int> PHs,
                                             cuda_tool::CBufferView<int> cnt,
                                             cuda_tool::CBufferView<Float> d0,
                                             cuda_tool::CBufferView<Vector3> d_grad,
                                             cuda_tool::CBufferView<Vector3> x,
                                             cuda_tool::BufferView<Float>    Es,
                                             int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_vertex_half_plane_contact;

        auto vI = PHs(idx);
        auto mu = mu_v(vI);
        auto c  = cnt(idx) >= 0 ? cnt(idx) : max(-cnt(idx) - 6, 0);
        Es(idx) =
            half_plane_penalty_energy(pow(decay, c) * mu, d0(idx), d_grad(idx), x(vI));
    }

    __global__ void do_assemble_kernel(cuda_tool::CBufferView<Float>   mu_v,
                                       Float                           decay,
                                       cuda_tool::CBufferView<int>     PHs,
                                       cuda_tool::CBufferView<int>     cnt,
                                       cuda_tool::CBufferView<Float>   d0,
                                       cuda_tool::CBufferView<Vector3> d_grad,
                                       cuda_tool::CBufferView<Vector3> x,
                                       cuda_tool::DoubletVectorView<Float, 3> Gs,
                                       cuda_tool::TripletMatrixView<Float, 3> Hs,
                                       int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_vertex_half_plane_contact;

        auto      vI = PHs(idx);
        auto      mu = mu_v(vI);
        Vector3   G;
        Matrix3x3 H;
        auto      c = cnt(idx) >= 0 ? cnt(idx) : max(-cnt(idx) - 6, 0);
        half_plane_penalty_gradient_hessian(
            pow(decay, c) * mu, d0(idx), d_grad(idx), x(vI), G, H);

        Gs(idx).write(vI, G);
        Hs(idx).write(vI, vI, H);
    }
}  // namespace

void ALVertexHalfPlaneNormalContact::do_build(ContactReporter::BuildInfo& info)
{
    require<ALIPCPipelineFlag>();
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

    info.gradient_count(active_set->PHs().size());
    info.hessian_count(active_set->PHs().size());
}

void ALVertexHalfPlaneNormalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace cuda_tool;
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

    if(PH_size > 0)
        do_compute_energy_kernel<<<cuda_tool::best_grid_dim((int)PH_size, do_compute_energy_kernel), cuda_tool::best_block_dim(do_compute_energy_kernel), 0, nullptr>>>(
            active_set->mu_vertices().cviewer(),
            active_set->decay_factor(),
            PHs.cviewer(),
            PH_cnt.cviewer(),
            PH_d0.cviewer(),
            PH_d_grad.cviewer(),
            x.cviewer(),
            PH_energies.viewer(),
            (int)PH_size);
}

void ALVertexHalfPlaneNormalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace cuda_tool;
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

    if(PH_size > 0)
        do_assemble_kernel<<<cuda_tool::best_grid_dim((int)PH_size, do_assemble_kernel), cuda_tool::best_block_dim(do_assemble_kernel), 0, nullptr>>>(
            active_set->mu_vertices().cviewer(),
            active_set->decay_factor(),
            PHs.cviewer(),
            PH_cnt.cviewer(),
            PH_d0.cviewer(),
            PH_d_grad.cviewer(),
            x.cviewer(),
            PH_grad.viewer(),
            PH_hess.viewer(),
            (int)PH_size);
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
