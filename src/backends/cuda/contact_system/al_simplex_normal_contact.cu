#include <contact_system/al_simplex_normal_contact.h>
#include <contact_system/al_contact_function.h>
#include <pipeline/al_ipc_pipeline_flag.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/log.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void do_compute_energy_k1_kernel(cuda_tool::CBufferView<Float> mu_v,
                                                Float decay,
                                                cuda_tool::CBufferView<Vector4i> PTs,
                                                cuda_tool::CBufferView<int> cnt,
                                                cuda_tool::CBufferView<Float> d0,
                                                cuda_tool::CBufferView<Vector12> d_grad,
                                                cuda_tool::CBufferView<Vector3> x,
                                                cuda_tool::BufferView<Float> Es,
                                                int                          n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto PT = PTs(idx);
        auto mu = min(min(mu_v(PT(0)), mu_v(PT(1))), min(mu_v(PT(2)), mu_v(PT(3))));
        auto c  = max(cnt(idx), 0);
        Es(idx) = penalty_energy(
            pow(decay, c) * mu, d0(idx), d_grad(idx), x(PT(0)), x(PT(1)), x(PT(2)), x(PT(3)));
    }

    __global__ void do_compute_energy_k2_kernel(cuda_tool::CBufferView<Float> mu_v,
                                                Float decay,
                                                cuda_tool::CBufferView<Vector4i> EEs,
                                                cuda_tool::CBufferView<int> cnt,
                                                cuda_tool::CBufferView<Float> d0,
                                                cuda_tool::CBufferView<Vector12> d_grad,
                                                cuda_tool::CBufferView<Vector3> x,
                                                cuda_tool::BufferView<Float> Es,
                                                int                          n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto EE = EEs(idx);
        auto mu = min(min(mu_v(EE(0)), mu_v(EE(1))), min(mu_v(EE(2)), mu_v(EE(3))));
        auto c  = max(cnt(idx), 0);
        Es(idx) = penalty_energy(
            pow(decay, c) * mu, d0(idx), d_grad(idx), x(EE(0)), x(EE(1)), x(EE(2)), x(EE(3)));
    }

    __global__ void do_assemble_k1_kernel(cuda_tool::CBufferView<Float> mu_v,
                                          Float                         decay,
                                          cuda_tool::CBufferView<Vector4i> PTs,
                                          cuda_tool::CBufferView<int>      cnt,
                                          cuda_tool::CBufferView<Float>    d0,
                                          cuda_tool::CBufferView<Vector12> d_grad,
                                          cuda_tool::CBufferView<Vector3> x,
                                          cuda_tool::DoubletVectorView<Float, 3> Gs,
                                          cuda_tool::TripletMatrixView<Float, 3> Hs,
                                          int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto PT = PTs(idx);
        auto mu = min(min(mu_v(PT(0)), mu_v(PT(1))), min(mu_v(PT(2)), mu_v(PT(3))));
        Vector12    G;
        Matrix12x12 H;
        auto        c = max(cnt(idx), 0);
        penalty_gradient_hessian(
            pow(decay, c) * mu, d0(idx), d_grad(idx), x(PT(0)), x(PT(1)), x(PT(2)), x(PT(3)), G, H);

        DoubletVectorAssembler DVA{Gs};
        DVA.segment<4>(idx * 4).write(PT, G);

        TripletMatrixAssembler TMA{Hs};
        TMA.half_block<4>(idx * 10).write(PT, H);
    }

    __global__ void do_assemble_k2_kernel(cuda_tool::CBufferView<Float> mu_v,
                                          Float                         decay,
                                          cuda_tool::CBufferView<Vector4i> EEs,
                                          cuda_tool::CBufferView<int>      cnt,
                                          cuda_tool::CBufferView<Float>    d0,
                                          cuda_tool::CBufferView<Vector12> d_grad,
                                          cuda_tool::CBufferView<Vector3> x,
                                          cuda_tool::DoubletVectorView<Float, 3> Gs,
                                          cuda_tool::TripletMatrixView<Float, 3> Hs,
                                          int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto EE = EEs(idx);
        auto mu = min(min(mu_v(EE(0)), mu_v(EE(1))), min(mu_v(EE(2)), mu_v(EE(3))));
        Vector12    G;
        Matrix12x12 H;
        auto        c = max(cnt(idx), 0);
        penalty_gradient_hessian(
            pow(decay, c) * mu, d0(idx), d_grad(idx), x(EE(0)), x(EE(1)), x(EE(2)), x(EE(3)), G, H);

        DoubletVectorAssembler DVA{Gs};
        DVA.segment<4>(idx * 4).write(EE, G);

        TripletMatrixAssembler TMA{Hs};
        TMA.half_block<4>(idx * 10).write(EE, H);
    }
}  // namespace

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

    if(PT_size > 0)
        do_compute_energy_k1_kernel<<<cuda_tool::best_grid_dim((int)PT_size, do_compute_energy_k1_kernel), cuda_tool::best_block_dim(do_compute_energy_k1_kernel), 0, nullptr>>>(
            active_set->mu_vertices().cviewer(),
            active_set->decay_factor(),
            PTs.cviewer(),
            PT_cnt.cviewer(),
            PT_d0.cviewer(),
            PT_d_grad.cviewer(),
            x.cviewer(),
            PT_energies.viewer(),
            (int)PT_size);

    if(EE_size > 0)
        do_compute_energy_k2_kernel<<<cuda_tool::best_grid_dim((int)EE_size, do_compute_energy_k2_kernel), cuda_tool::best_block_dim(do_compute_energy_k2_kernel), 0, nullptr>>>(
            active_set->mu_vertices().cviewer(),
            active_set->decay_factor(),
            EEs.cviewer(),
            EE_cnt.cviewer(),
            EE_d0.cviewer(),
            EE_d_grad.cviewer(),
            x.cviewer(),
            EE_energies.viewer(),
            (int)EE_size);
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

    if(PT_size > 0)
        do_assemble_k1_kernel<<<cuda_tool::best_grid_dim((int)PT_size, do_assemble_k1_kernel), cuda_tool::best_block_dim(do_assemble_k1_kernel), 0, nullptr>>>(
            active_set->mu_vertices().cviewer(),
            active_set->decay_factor(),
            PTs.cviewer(),
            PT_cnt.cviewer(),
            PT_d0.cviewer(),
            PT_d_grad.cviewer(),
            x.cviewer(),
            PT_grad.viewer(),
            PT_hess.viewer(),
            (int)PT_size);

    if(EE_size > 0)
        do_assemble_k2_kernel<<<cuda_tool::best_grid_dim((int)EE_size, do_assemble_k2_kernel), cuda_tool::best_block_dim(do_assemble_k2_kernel), 0, nullptr>>>(
            active_set->mu_vertices().cviewer(),
            active_set->decay_factor(),
            EEs.cviewer(),
            EE_cnt.cviewer(),
            EE_d0.cviewer(),
            EE_d_grad.cviewer(),
            x.cviewer(),
            EE_grad.viewer(),
            EE_hess.viewer(),
            (int)EE_size);
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
