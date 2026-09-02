#include <contact_system/al_contact_function.h>
#include <contact_system/al_simplex_frictional_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <utils/matrix_assembler.h>
#include <uipc/common/log.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void do_compute_energy_k1_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                Float eps_v,
                                                Float dt,
                                                cuda_tool::CBufferView<Vector4i> PTs,
                                                cuda_tool::CBufferView<Float> lambda,
                                                cuda_tool::CBufferView<Vector3> x,
                                                cuda_tool::CBufferView<Vector3> prev_x,
                                                cuda_tool::BufferView<Float> Es,
                                                int                          n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto     PT   = PTs(idx);
        Vector4i cids = {contact_ids(PT[0]),
                         contact_ids(PT[1]),
                         contact_ids(PT[2]),
                         contact_ids(PT[3])};

        auto  coeff = sym::codim_ipc_contact::PT_contact_coeff(table, cids);
        Float mu    = coeff.mu;

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
    }

    __global__ void do_compute_energy_k2_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                Float eps_v,
                                                Float dt,
                                                cuda_tool::CBufferView<Vector4i> EEs,
                                                cuda_tool::CBufferView<Float> lambda,
                                                cuda_tool::CBufferView<Vector3> x,
                                                cuda_tool::CBufferView<Vector3> rest_x,
                                                cuda_tool::CBufferView<Vector3> prev_x,
                                                cuda_tool::BufferView<Float> Es,
                                                int                          n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto     EE   = EEs(idx);
        Vector4i cids = {contact_ids(EE[0]),
                         contact_ids(EE[1]),
                         contact_ids(EE[2]),
                         contact_ids(EE[3])};

        auto  coeff = sym::codim_ipc_contact::EE_contact_coeff(table, cids);
        Float mu    = coeff.mu;

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

        if(EE_friction_mollified(
               rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1))
        {
            Es(idx) = 0.0;
            return;
        }

        Es(idx) = EE_friction_energy(
            mu, eps_v * dt, lambda(idx), prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
    }

    __global__ void do_assemble_k1_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                          cuda_tool::CBufferView<IndexT> contact_ids,
                                          Float eps_v,
                                          Float dt,
                                          cuda_tool::CBufferView<Vector4i> PTs,
                                          cuda_tool::CBufferView<Float> lambda,
                                          cuda_tool::CBufferView<Vector3> x,
                                          cuda_tool::CBufferView<Vector3> prev_x,
                                          cuda_tool::DoubletVectorView<Float, 3> Gs,
                                          cuda_tool::TripletMatrixView<Float, 3> Hs,
                                          int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto     PT   = PTs(idx);
        Vector4i cids = {contact_ids(PT[0]),
                         contact_ids(PT[1]),
                         contact_ids(PT[2]),
                         contact_ids(PT[3])};

        auto  coeff = sym::codim_ipc_contact::PT_contact_coeff(table, cids);
        Float mu    = coeff.mu;

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

        TripletMatrixAssembler TMA{Hs};
        TMA.half_block<4>(idx * 10).write(PT, H);
    }

    __global__ void do_assemble_k2_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                          cuda_tool::CBufferView<IndexT> contact_ids,
                                          Float eps_v,
                                          Float dt,
                                          cuda_tool::CBufferView<Vector4i> EEs,
                                          cuda_tool::CBufferView<Float> lambda,
                                          cuda_tool::CBufferView<Vector3> x,
                                          cuda_tool::CBufferView<Vector3> rest_x,
                                          cuda_tool::CBufferView<Vector3> prev_x,
                                          cuda_tool::DoubletVectorView<Float, 3> Gs,
                                          cuda_tool::TripletMatrixView<Float, 3> Hs,
                                          int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::al_simplex_contact;

        auto     EE   = EEs(idx);
        Vector4i cids = {contact_ids(EE[0]),
                         contact_ids(EE[1]),
                         contact_ids(EE[2]),
                         contact_ids(EE[3])};

        auto  coeff = sym::codim_ipc_contact::EE_contact_coeff(table, cids);
        Float mu    = coeff.mu;

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

        const bool mollified = EE_friction_mollified(
            rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1);

        Vector12    G = Vector12::Zero();
        Matrix12x12 H = Matrix12x12::Zero();
        if(!mollified)
        {
            EE_friction_gradient_hessian(
                G, H, mu, eps_v * dt, lambda(idx), prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
        }

        DoubletVectorAssembler DVA{Gs};
        DVA.segment<4>(idx * 4).write(EE, G);

        TripletMatrixAssembler TMA{Hs};
        TMA.half_block<4>(idx * 10).write(EE, H);
    }
}  // namespace

void ALSimplexFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_contact_manager = require<GlobalContactManager>();
    m_impl.global_vertex_manager  = require<GlobalVertexManager>();
    m_impl.global_surf_manager    = require<GlobalSimplicialSurfaceManager>();
    m_impl.global_active_set_manager = require<GlobalActiveSetManager>();

    m_impl.dt_attr = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");
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

    info.gradient_count(
        4 * (active_set->PTs_friction().size() + active_set->EEs_friction().size()));
    info.hessian_count(
        10 * (active_set->PTs_friction().size() + active_set->EEs_friction().size()));
}

void ALSimplexFrictionalContact::Impl::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace cuda_tool;
    using namespace sym::al_simplex_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PT_size     = active_set->PTs_friction().size();
    auto EE_size     = active_set->EEs_friction().size();
    auto PT_energies = info.energies().subview(0, PT_size);
    auto EE_energies = info.energies().subview(PT_size, EE_size);

    auto x         = global_vertex_manager->positions();
    auto rest_x    = global_vertex_manager->rest_positions();
    auto prev_x    = global_vertex_manager->prev_positions();
    auto PTs       = active_set->PTs_friction();
    auto EEs       = active_set->EEs_friction();
    auto PT_lambda = active_set->PT_lambda_friction();
    auto EE_lambda = active_set->EE_lambda_friction();

    if(PT_size > 0)
        do_compute_energy_k1_kernel<<<cuda_tool::best_grid_dim((int)PT_size, do_compute_energy_k1_kernel), cuda_tool::best_block_dim(do_compute_energy_k1_kernel), 0, nullptr>>>(
            global_contact_manager->contact_tabular().cviewer(),
            global_vertex_manager->contact_element_ids().cviewer(),
            global_contact_manager->eps_velocity(),
            dt_attr->view()[0],
            PTs.cviewer(),
            PT_lambda.cviewer(),
            x.cviewer(),
            prev_x.cviewer(),
            PT_energies.viewer(),
            (int)PT_size);

    if(EE_size > 0)
        do_compute_energy_k2_kernel<<<cuda_tool::best_grid_dim((int)EE_size, do_compute_energy_k2_kernel), cuda_tool::best_block_dim(do_compute_energy_k2_kernel), 0, nullptr>>>(
            global_contact_manager->contact_tabular().cviewer(),
            global_vertex_manager->contact_element_ids().cviewer(),
            global_contact_manager->eps_velocity(),
            dt_attr->view()[0],
            EEs.cviewer(),
            EE_lambda.cviewer(),
            x.cviewer(),
            rest_x.cviewer(),
            prev_x.cviewer(),
            EE_energies.viewer(),
            (int)EE_size);
}

void ALSimplexFrictionalContact::Impl::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    using namespace cuda_tool;
    using namespace sym::al_simplex_contact;
    auto& active_set = global_active_set_manager;

    if(!active_set->is_enabled())
        return;

    auto PT_size = active_set->PTs_friction().size();
    auto EE_size = active_set->EEs_friction().size();
    auto PT_grad = info.gradients().subview(0, PT_size * 4);
    auto EE_grad = info.gradients().subview(PT_size * 4, EE_size * 4);
    auto PT_hess = info.hessians().subview(0, PT_size * 10);
    auto EE_hess = info.hessians().subview(PT_size * 10, EE_size * 10);

    auto x         = global_vertex_manager->positions();
    auto prev_x    = global_vertex_manager->prev_positions();
    auto rest_x    = global_vertex_manager->rest_positions();
    auto PTs       = active_set->PTs_friction();
    auto EEs       = active_set->EEs_friction();
    auto PT_lambda = active_set->PT_lambda_friction();
    auto EE_lambda = active_set->EE_lambda_friction();

    if(PT_size > 0)
        do_assemble_k1_kernel<<<cuda_tool::best_grid_dim((int)PT_size, do_assemble_k1_kernel), cuda_tool::best_block_dim(do_assemble_k1_kernel), 0, nullptr>>>(
            global_contact_manager->contact_tabular().cviewer(),
            global_vertex_manager->contact_element_ids().cviewer(),
            global_contact_manager->eps_velocity(),
            dt_attr->view()[0],
            PTs.cviewer(),
            PT_lambda.cviewer(),
            x.cviewer(),
            prev_x.cviewer(),
            PT_grad.viewer(),
            PT_hess.viewer(),
            (int)PT_size);

    if(EE_size > 0)
        do_assemble_k2_kernel<<<cuda_tool::best_grid_dim((int)EE_size, do_assemble_k2_kernel), cuda_tool::best_block_dim(do_assemble_k2_kernel), 0, nullptr>>>(
            global_contact_manager->contact_tabular().cviewer(),
            global_vertex_manager->contact_element_ids().cviewer(),
            global_contact_manager->eps_velocity(),
            dt_attr->view()[0],
            EEs.cviewer(),
            EE_lambda.cviewer(),
            x.cviewer(),
            rest_x.viewer(),
            prev_x.cviewer(),
            EE_grad.viewer(),
            EE_hess.viewer(),
            (int)EE_size);
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
