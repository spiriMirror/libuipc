#include <affine_body/abd_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/matrix_assembler.h>
#include <utils/matrix_unpacker.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constitution_manager.h>
#include <affine_body/abd_linear_subsystem_reporter.h>
#include <affine_body/affine_body_kinetic.h>
#include <affine_body/affine_body_constitution.h>
#include <utils/report_extent_check.h>

namespace uipc::backend::cuda
{
UIPC_GENERIC void zero_out_lower(Matrix12x12& H)
{
    // clear lower triangle (3x3 block based)
    for(IndexT jj = 0; jj < 4; ++jj)
    {
        for(IndexT ii = jj + 1; ii < 4; ++ii)
        {
            H.block<3, 3>(ii * 3, jj * 3).setZero();
        }
    }
}

namespace
{
    using namespace cuda_tool;  // for eigen:: / view aliases used inside kernel bodies

    __global__ void abd_linear_subsystem_assemble_kinetic_shape_k1_kernel(
        cuda_tool::CBufferView<IndexT>    is_fixed,
        cuda_tool::CBufferView<IndexT>    is_external_kinetic,
        cuda_tool::CBufferView<Vector12>  shape_gradient,
        cuda_tool::CBufferView<Vector12>  kinetic_gradient,
        cuda_tool::DenseVectorView<Float> gradients,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector12 src;

        if(is_fixed(i))
        {
            src.setZero();  // if fixed, set to zero
        }
        else
        {
            src = shape_gradient(i);

            // if not external kinetic, add kinetic gradient
            if(!is_external_kinetic(i)) [[likely]]
            {
                src += kinetic_gradient(i);
            }
        }

        gradients.segment<12>(i * 12) = src;

        // cout << "EKG(" << i << "): " << src.transpose().eval() << "\n";
    }

    __global__ void abd_linear_subsystem_assemble_kinetic_shape_k2_kernel(
        cuda_tool::TripletMatrixView<Float, 3> dst,
        cuda_tool::CBufferView<IndexT>         is_fixed,
        cuda_tool::CBufferView<IndexT>         is_external_kinetic,
        cuda_tool::CBufferView<Matrix12x12>    shape_hessian,
        cuda_tool::CBufferView<Matrix12x12>    kinetic_hessian,
        cuda_tool::BufferView<Matrix12x12>     diag_hessian,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        TripletMatrixUnpacker MA{dst};
        Matrix12x12           H12x12;

        if(is_fixed(I))
        {
            // Fill kinetic hessian to identity to avoid singularity
            H12x12.setIdentity();
        }
        else
        {
            // if not fixed, fill shape hessian
            H12x12 = shape_hessian(I);

            // if not external kinetic, add kinetic gradient
            if(!is_external_kinetic(I)) [[likely]]
            {
                H12x12 += kinetic_hessian(I);
            }
        }

        // record diagonal hessian for diag-inv preconditioner
        diag_hessian(I) = H12x12;

        // set the lower triangle blocks to zero for robustness
        zero_out_lower(H12x12);

        MA.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
            .write(I * 4,          // begin row
                   I * 4,          // begin col
                   H12x12);
    }

    __global__ void abd_linear_subsystem_assemble_reporters_k1_kernel(
        cuda_tool::DenseVectorView<Float>        dst,
        cuda_tool::CDoubletVectorView<Float, 12> src,
        cuda_tool::CBufferView<IndexT>           is_fixed,
        int                                      n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [body_i, G12] = src(I);

        if(is_fixed(body_i))
        {
            // Do nothing
        }
        else
        {
            dst.segment<12>(body_i * 12).atomic_add(G12);
        }
    }

    __global__ void abd_linear_subsystem_assemble_reporters_k2_kernel(
        cuda_tool::TripletMatrixView<Float, 3>       dst,
        cuda_tool::CTripletMatrixView<Float, 12, 12> src,
        cuda_tool::BufferView<Matrix12x12>           diag_hessian,
        cuda_tool::CBufferView<IndexT>               is_fixed,
        int                                          n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        TripletMatrixUnpacker MU{dst};
        Matrix12x12           H12x12;
        auto&& [body_i, body_j, Value] = src(I);
        H12x12                         = Value;

        bool has_fixed = (is_fixed(body_i) || is_fixed(body_j));

        // Fill diagonal hessian for diag-inv preconditioner
        if(body_i == body_j && !has_fixed)
        {
            eigen::atomic_add(diag_hessian(body_i), H12x12);
        }

        if(has_fixed)
        {
            // Zero out hessian for fixed bodies
            H12x12.setZero();
        }
        else
        {
            if(body_i == body_j)
            {
                // Since body_i == body_j, we only fill the upper triangle part
                zero_out_lower(H12x12);
            }
            else if(body_i > body_j)
            {
                // If all the reporters only report upper triangle part, this branch should not be hit
                H12x12.setZero();
            }
        }

        MU.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
            .write(body_i * 4,     // begin row
                   body_j * 4,     // begin col
                   H12x12);
    }

    __global__ void abd_linear_subsystem_assemble_dytopo_effect_k1_kernel(
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        cuda_tool::DenseVectorView<Float>       gradients,
        cuda_tool::CBufferView<IndexT>          v2b,
        cuda_tool::CBufferView<ABDJacobi>       Js,
        cuda_tool::CBufferView<IndexT>          is_fixed,
        IndexT                                  vertex_offset,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, G3] = dytopo_effect_gradient(I);

        auto  i      = g_i - vertex_offset;
        auto  body_i = v2b(i);
        auto& J_i    = Js(i);

        if(is_fixed(body_i))
        {
            // Do nothing
        }
        else
        {
            Vector12 G12 = J_i.T() * G3;
            gradients.segment<12>(body_i * 12).atomic_add(G12);

            // cout << "DG(" << I << "): " << G12.transpose().eval() << "\n";
        }
    }

    __global__ void abd_linear_subsystem_assemble_dytopo_effect_k2_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        cuda_tool::TripletMatrixView<Float, 3>     dst,
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::CBufferView<ABDJacobi>          Js,
        cuda_tool::CBufferView<IndexT>             is_fixed,
        cuda_tool::BufferView<Matrix12x12>         diag_hessian,
        IndexT                                     vertex_offset,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, g_j, H3x3] = dytopo_effect_hessian(I);

        auto i = g_i - vertex_offset;
        auto j = g_j - vertex_offset;

        auto body_i = v2b(i);
        auto body_j = v2b(j);

        auto& J_i = Js(i);
        auto& J_j = Js(j);

        Matrix12x12 H12x12;

        // We know half contact hessian i <= j
        // but we don't know body_i and body_j order
        // so test and swap if necessary
        IndexT L = body_i;
        IndexT R = body_j;
        if(body_i > body_j)
        {
            L = body_j;
            R = body_i;
        }

        if(is_fixed(body_i) || is_fixed(body_j))
        {
            H12x12.setZero();
        }
        else
        {
            if(body_i < body_j)
            {
                H12x12 = ABDJacobi::JT_H_J(J_i.T(), H3x3, J_j);
            }
            else if(body_i > body_j)
            {
                H12x12 = ABDJacobi::JT_H_J(J_j.T(), H3x3.transpose(), J_i);
            }
            else  // body_i == body_j
            {
                // Two vertices from the same body
                if(i != j)
                {
                    H12x12 = ABDJacobi::JT_H_J(J_i.T(), H3x3, J_j)
                             + ABDJacobi::JT_H_J(J_j.T(), H3x3.transpose(), J_i);
                }
                else  // i == j
                {
                    H12x12 = ABDJacobi::JT_H_J(J_i.T(), H3x3, J_j);
                }

                // Fill diagonal hessian for diag-inv preconditioner
                eigen::atomic_add(diag_hessian(body_i), H12x12);

                // Since body_i == body_j, we only fill the upper triangle part
                zero_out_lower(H12x12);
            }
        }

        TripletMatrixUnpacker MU{dst};
        MU.block<4, 4>(I * 4 * 4)  // triplet range of [I*16, (I+1)*16)
            .write(L * 4,          // begin row
                   R * 4,          // begin col
                   H12x12);
    }

    __global__ void abd_linear_subsystem_retrieve_solution_kernel(
        cuda_tool::BufferView<Vector12> dq, cuda_tool::CDenseVectorView<Float> x, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // retrieve solution for each body
        dq(i) = -x.segment<12>(i * 12).as_eigen();
    }

    __global__ void abd_linear_subsystem_diag_norm_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_hess,
        cuda_tool::BufferView<Float>        diag_blocks_norm,
        cuda_tool::CBufferView<IndexT>      is_fixed,
        int                                 n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        for(int i = 0; i < 12; i++)
            diag_blocks_norm(idx * 12 + i) =
                is_fixed(idx) ? 0 : abs(diag_hess(idx)(i, i));
    }

    __global__ void abd_linear_subsystem_mass_norm_kernel(
        cuda_tool::CBufferView<ABDJacobiDyadicMass> mass,
        cuda_tool::BufferView<Float>                block_norm,
        cuda_tool::CBufferView<IndexT>              is_fixed,
        int                                         n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        block_norm(idx) = is_fixed(idx) ? 0 : mass(idx).mass();
    }
}  // namespace
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDLinearSubsystem);

// ref: https://github.com/spiriMirror/libuipc/issues/271
constexpr U64 ABDLinearSubsystemUID = 0ull;

void ABDLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo& info)
{
    m_impl.affine_body_dynamics        = require<AffineBodyDynamics>();
    m_impl.affine_body_vertex_reporter = require<AffineBodyVertexReporter>();
    m_impl.dt_attr = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");

    m_impl.dytopo_effect_receiver = find<ABDDyTopoEffectReceiver>();
}

void ABDLinearSubsystem::Impl::init()
{
    auto reporter_view = reporters.view();
    for(auto&& [i, r] : enumerate(reporter_view))
        r->m_index = i;
    for(auto& r : reporter_view)
        r->init();

    reporter_gradient_offsets_counts.resize(reporter_view.size());
    reporter_hessian_offsets_counts.resize(reporter_view.size());

    SizeT body_count = abd().body_count();
    body_id_to_shape_hessian.resize(body_count);
    body_id_to_shape_gradient.resize(body_count);
    body_id_to_kinetic_hessian.resize(body_count);
    body_id_to_kinetic_gradient.resize(body_count);
    diag_hessian.resize(body_count);
}

void ABDLinearSubsystem::Impl::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    info.extent(abd().body_count() * 12);
}

void ABDLinearSubsystem::Impl::receive_init_dof_info(WorldVisitor& w,
                                                     GlobalLinearSystem::InitDofInfo& info)
{
    auto& geo_infos = abd().geo_infos;
    auto  geo_slots = w.scene().geometries();

    IndexT offset = info.dof_offset();

    // fill the dof_offset and dof_count for each geometry
    affine_body_dynamics->for_each(
        geo_slots,
        [&](const AffineBodyDynamics::ForEachInfo& foreach_info, geometry::SimplicialComplex& sc)
        {
            auto I          = foreach_info.global_index();
            auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
            UIPC_ASSERT(dof_offset, "dof_offset not found on ABD mesh why can it happen?");
            auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
            UIPC_ASSERT(dof_count, "dof_count not found on ABD mesh why can it happen?");

            IndexT this_dof_count = 12 * sc.instances().size();
            view(*dof_offset)[0]  = offset;
            view(*dof_count)[0]   = this_dof_count;

            offset += this_dof_count;
        });

    UIPC_ASSERT(offset == info.dof_offset() + info.dof_count(), "dof size mismatch");
}

void ABDLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    // 1. Gradient Count
    constexpr SizeT G12_to_dof = 12;
    SizeT           body_count = abd().body_count();
    auto            dof_count  = body_count * G12_to_dof;

    auto has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    SizeT H12x12_count = 0;

    if(has_complement)
    {
        // 1) Body hessian: kinetic + shape
        if(!info.gradient_only())
            H12x12_count += abd().body_count();

        // 2) Reporters
        auto reporter_view = reporters.view();
        auto grad_counts   = reporter_gradient_offsets_counts.counts();
        auto hess_counts   = reporter_hessian_offsets_counts.counts();

        for(auto&& R : reporter_view)
        {
            ReportExtentInfo extent_info;
            extent_info.m_gradient_only = info.gradient_only();
            R->report_extent(extent_info);

            grad_counts[R->m_index] = extent_info.m_gradient_count;
            hess_counts[R->m_index] = extent_info.m_hessian_count;
        }

        reporter_gradient_offsets_counts.scan();
        reporter_hessian_offsets_counts.scan();

        if(!info.gradient_only())
            H12x12_count += reporter_hessian_offsets_counts.total_count();
    }


    if(dytopo_effect_receiver && !info.gradient_only())
    {
        H12x12_count += dytopo_effect_receiver->hessians().triplet_count();
    }


    auto H3x3_count = H12x12_count * (4 * 4);

    if(info.gradient_only())
    {
        UIPC_ASSERT(H3x3_count == 0,
                    "Hessian block count should be zero (got {}) when gradient_only is true",
                    H3x3_count);
    }

    info.extent(H3x3_count, dof_count);
}

void ABDLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // 0) Prepare buffers for reporters
    {
        auto N = abd().body_count();

        reporter_gradients.reshape(N);
        reporter_gradients.resize_doublets(reporter_gradient_offsets_counts.total_count());

        reporter_hessians.reshape(N, N);
        reporter_hessians.resize_triplets(reporter_hessian_offsets_counts.total_count());
    }

    bool has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    IndexT hess_offset = 0;

    // 1) Static Topo Effect: Kinetic + Shape + Other Reporters
    if(has_complement)
    {
        _assemble_kinetic_shape(hess_offset, info);
        _assemble_reporters(hess_offset, info);
    }
    else  // contact only
    {
        info.gradients().buffer_view().fill(0);
    }

    // 2) Dynamic Topology Effect
    _assemble_dytopo_effect(hess_offset, info);

    UIPC_ASSERT(hess_offset == info.hessians().triplet_count(),
                "Hessian size mismatch: expected {}, got {}",
                info.hessians().triplet_count(),
                hess_offset);
}

void ABDLinearSubsystem::Impl::_assemble_kinetic_shape(IndexT& hess_offset,
                                                       GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    Float dt = dt_attr->view()[0];

    // Collect Kinetic
    ABDLinearSubsystem::ComputeGradientHessianInfo this_info{
        info.gradient_only(), body_id_to_kinetic_gradient, body_id_to_kinetic_hessian, dt};
    abd().kinetic->compute_gradient_hessian(this_info);

    // Collect Shape
    for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
    {
        ABDLinearSubsystem::ComputeGradientHessianInfo this_info{
            info.gradient_only(),
            abd().subview(body_id_to_shape_gradient, cst->m_index),
            abd().subview(body_id_to_shape_hessian, cst->m_index),
            dt};

        cst->compute_gradient_hessian(this_info);
    }

    {
        auto k = abd_linear_subsystem_assemble_kinetic_shape_k1_kernel;
        int  n = (int)abd().body_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                abd().body_id_to_is_fixed.cview(),
                abd().body_id_to_external_kinetic.cview(),
                body_id_to_shape_gradient.cview(),
                body_id_to_kinetic_gradient.cview(),
                info.gradients(),
                n);
    }

    if(info.gradient_only())
        return;

    auto body_count = body_id_to_shape_hessian.size();
    auto H3x3_count = body_count * (4 * 4);
    auto body_H3x3  = info.hessians().subview(hess_offset, H3x3_count);

    {
        auto k = abd_linear_subsystem_assemble_kinetic_shape_k2_kernel;
        int  n = (int)body_count;
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                body_H3x3,
                abd().body_id_to_is_fixed.cview(),
                abd().body_id_to_external_kinetic.cview(),
                body_id_to_shape_hessian.cview(),
                body_id_to_kinetic_hessian.cview(),
                diag_hessian.view(),
                n);
    }

    hess_offset += H3x3_count;
}

void ABDLinearSubsystem::Impl::_assemble_reporters(IndexT& offset,
                                                   GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // Fill TripletMatrix and DoubletVector
    for(auto& R : reporters.view())
    {
        AssembleInfo assemble_info{this, R->m_index, info.gradient_only()};
        R->assemble(assemble_info);
    }

    if(reporter_gradients.doublet_count())
    {
        auto k = abd_linear_subsystem_assemble_reporters_k1_kernel;
        int  n = (int)reporter_gradients.doublet_count();
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.gradients(),
            reporter_gradients.cview(),
            abd().body_id_to_is_fixed.cview(),
            n);
    }

    if(!info.gradient_only() && reporter_hessians.triplet_count())
    {
        // get rest
        auto H3x3s = info.hessians().subview(offset);

        auto k = abd_linear_subsystem_assemble_reporters_k2_kernel;
        int  n = (int)reporter_hessians.triplet_count();
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            H3x3s,
            reporter_hessians.cview(),
            diag_hessian.view(),
            abd().body_id_to_is_fixed.cview(),
            n);

        offset += reporter_hessians.triplet_count() * (4 * 4);
    }
}

void ABDLinearSubsystem::Impl::_assemble_dytopo_effect(IndexT& offset,
                                                       GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    auto  vertex_offset = affine_body_vertex_reporter->vertex_offset();
    SizeT dytopo_effect_gradient_count = 0;
    if(dytopo_effect_receiver)
    {
        dytopo_effect_gradient_count =
            dytopo_effect_receiver->gradients().doublet_count();
    }

    if(dytopo_effect_gradient_count)
    {
        auto k = abd_linear_subsystem_assemble_dytopo_effect_k1_kernel;
        int  n = (int)dytopo_effect_gradient_count;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dytopo_effect_receiver->gradients(),
            info.gradients(),
            abd().vertex_id_to_body_id.cview(),
            abd().vertex_id_to_J.cview(),
            abd().body_id_to_is_fixed.cview(),
            vertex_offset,
            n);
    }

    if(info.gradient_only())
        return;

    SizeT dytopo_effect_hessian_count = 0;
    if(dytopo_effect_receiver)
        dytopo_effect_hessian_count = dytopo_effect_receiver->hessians().triplet_count();

    auto H3x3_count         = dytopo_effect_hessian_count * (4 * 4);
    auto dytopo_effect_H3x3 = info.hessians().subview(offset, H3x3_count);

    if(dytopo_effect_hessian_count)
    {
        // Half Contact Hessian
        // ref: https://github.com/spiriMirror/libuipc/issues/272
        auto k = abd_linear_subsystem_assemble_dytopo_effect_k2_kernel;
        int  n = (int)dytopo_effect_hessian_count;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dytopo_effect_receiver->hessians(),
            dytopo_effect_H3x3,
            abd().vertex_id_to_body_id.cview(),
            abd().vertex_id_to_J.cview(),
            abd().body_id_to_is_fixed.cview(),
            diag_hessian.view(),
            vertex_offset,
            n);
    }

    offset += H3x3_count;
}

void ABDLinearSubsystem::Impl::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    info.satisfied(true);
}

void ABDLinearSubsystem::Impl::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    using namespace cuda_tool;

    auto dq = abd().body_id_to_dq.view();
    {
        auto k = abd_linear_subsystem_retrieve_solution_kernel;
        int  n = (int)abd().body_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                dq, info.solution(), n);
    }
}

Float ABDLinearSubsystem::Impl::diag_norm()
{
    auto diag_hess = diag_hessian.view();
    block_norm.resize(diag_hess.size() * 12);
    {
        auto k = abd_linear_subsystem_diag_norm_kernel;
        int  n = (int)diag_hess.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                diag_hess.cview(), block_norm.view(), abd().body_id_to_is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(
        block_norm.data(), reduced_norm.data(), block_norm.size());

    return reduced_norm;
}

Float ABDLinearSubsystem::Impl::mass_norm()
{
    auto mass = abd().body_id_to_abd_mass.view();
    block_norm.resize(mass.size());
    {
        auto k = abd_linear_subsystem_mass_norm_kernel;
        int  n = (int)mass.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mass.cview(), block_norm.view(), abd().body_id_to_is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(
        block_norm.data(), reduced_norm.data(), block_norm.size());

    return reduced_norm;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void ABDLinearSubsystem::do_init(InitInfo& info)
{
    m_impl.init();
}

void ABDLinearSubsystem::do_report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    m_impl.report_extent(info);
}

void ABDLinearSubsystem::do_assemble(GlobalLinearSystem::DiagInfo& info)
{
    m_impl.assemble(info);
}

void ABDLinearSubsystem::do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    m_impl.accuracy_check(info);
}

void ABDLinearSubsystem::do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    m_impl.retrieve_solution(info);
}

Float ABDLinearSubsystem::do_diag_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.diag_norm();
}

Float ABDLinearSubsystem::do_mass_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.mass_norm();
}

U64 ABDLinearSubsystem::get_uid() const noexcept
{
    return ABDLinearSubsystemUID;
}

void ABDLinearSubsystem::add_reporter(ABDLinearSubsystemReporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter cannot be null");
    check_state(SimEngineState::BuildSystems, "add_reporter");
    m_impl.reporters.register_sim_system(*reporter);
}

void ABDLinearSubsystem::do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    m_impl.report_init_extent(info);
}

void ABDLinearSubsystem::do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    m_impl.receive_init_dof_info(world(), info);
}

ABDLinearSubsystem::AssembleInfo::AssembleInfo(Impl* impl, IndexT index, bool gradient_only) noexcept
    : m_impl(impl)
    , m_index(index)
    , m_gradient_only(gradient_only)
{
}

cuda_tool::DoubletVectorView<Float, 12> ABDLinearSubsystem::AssembleInfo::gradients() const
{
    auto [offset, count] = m_impl->reporter_gradient_offsets_counts[m_index];
    return m_impl->reporter_gradients.view().subview(offset, count);
}

cuda_tool::TripletMatrixView<Float, 12, 12> ABDLinearSubsystem::AssembleInfo::hessians() const
{
    auto [offset, count] = m_impl->reporter_hessian_offsets_counts[m_index];
    return m_impl->reporter_hessians.view().subview(offset, count);
}

bool ABDLinearSubsystem::AssembleInfo::gradient_only() const noexcept
{
    return m_gradient_only;
}

void ABDLinearSubsystem::ReportExtentInfo::gradient_count(SizeT size)
{
    m_gradient_count = size;
}

void ABDLinearSubsystem::ReportExtentInfo::hessian_count(SizeT size)
{
    m_hessian_count = size;
}

void ABDLinearSubsystem::ReportExtentInfo::check(std::string_view name) const
{
    check_report_extent(m_gradient_only_checked, m_gradient_only, m_hessian_count, name);
}

AffineBodyDynamics::Impl& ABDLinearSubsystem::Impl::abd() const noexcept
{
    return affine_body_dynamics->m_impl;
}
}  // namespace uipc::backend::cuda
