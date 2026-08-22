#include <linear_system/off_diag_linear_subsystem.h>
#include <coupling_system/abd_fem_dytopo_effect_receiver.h>
#include <coupling_system/fem_abd_dytopo_effect_receiver.h>
#include <affine_body/abd_linear_subsystem.h>
#include <finite_element/fem_linear_subsystem.h>
#include <linear_system/global_linear_system.h>
#include <affine_body/affine_body_dynamics.h>
#include <finite_element/finite_element_method.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <kernel_cout.h>
#include <utils/matrix_unpacker.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void ABDFEMLinearSubsystem_assemble_kernel(
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::CBufferView<ABDJacobi>          Js,
        cuda_tool::CBufferView<IndexT>             body_is_fixed,
        cuda_tool::CBufferView<IndexT>             vertex_is_fixed,
        cuda_tool::TripletMatrixView<Float, 3, 3>  L,
        cuda_tool::CTripletMatrixView<Float, 3, 3> abd_fem_dytopo_effect,
        IndexT                                     abd_point_offset,
        IndexT                                     fem_point_offset,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        {
            // global vertex indices
            auto&& [gI_abd_v, gJ_fem_v, H3x3] = abd_fem_dytopo_effect(I);

            UIPC_KERNEL_ASSERT(abd_point_offset <= fem_point_offset,
                               "We assume ABD vertices are before FEM vertices, "
                               "but got abd_point_offset=%d, fem_point_offset=%d",
                               abd_point_offset,
                               fem_point_offset);

            auto I_abd_v = gI_abd_v - abd_point_offset;
            auto J_fem_v = gJ_fem_v - fem_point_offset;

            auto body_id = v2b(I_abd_v);

            const ABDJacobi& J = Js(I_abd_v);

            auto I4 = 4 * I;

            Matrix12x3 H = J.to_mat().transpose() * H3x3;

            if(body_is_fixed(body_id) || vertex_is_fixed(J_fem_v))
                H.setZero();

            //for(int k = 0; k < 4; ++k)
            //{
            //    L(I4 + k).write(4 * body_id + k,  // abd
            //                    J_fem_v,          // fem
            //                    H.block<3, 3>(3 * k, 0));
            //}

            TripletMatrixUnpacker TMU{L};

            TMU.block<4, 1>(I4).write(4 * body_id, J_fem_v, H);
        }
    }
}  // namespace

class ABDFEMLinearSubsystem final : public OffDiagLinearSubsystem
{
  public:
    using OffDiagLinearSubsystem::OffDiagLinearSubsystem;

    SimSystemSlot<GlobalLinearSystem> global_linear_system;

    SimSystemSlot<ABDFEMDyTopoEffectReceiver> abd_fem_dytopo_effect_receiver;

    SimSystemSlot<ABDLinearSubsystem> abd_linear_subsystem;
    SimSystemSlot<FEMLinearSubsystem> fem_linear_subsystem;

    SimSystemSlot<AffineBodyDynamics>  affine_body_dynamics;
    SimSystemSlot<FiniteElementMethod> finite_element_method;

    SimSystemSlot<AffineBodyVertexReporter>    affine_body_vertex_reporter;
    SimSystemSlot<FiniteElementVertexReporter> finite_element_vertex_reporter;

    virtual void do_build(BuildInfo& info) override
    {
        global_linear_system = require<GlobalLinearSystem>();

        abd_fem_dytopo_effect_receiver = require<ABDFEMDyTopoEffectReceiver>();

        abd_linear_subsystem = require<ABDLinearSubsystem>();
        fem_linear_subsystem = require<FEMLinearSubsystem>();

        affine_body_dynamics  = require<AffineBodyDynamics>();
        finite_element_method = require<FiniteElementMethod>();

        affine_body_vertex_reporter    = require<AffineBodyVertexReporter>();
        finite_element_vertex_reporter = require<FiniteElementVertexReporter>();

        info.connect(abd_linear_subsystem.view(), fem_linear_subsystem.view());
    }

    virtual void report_extent(GlobalLinearSystem::OffDiagExtentInfo& info) override
    {
        if(!abd_fem_dytopo_effect_receiver)
        {
            info.extent(0, 0);
            return;
        }

        // ABD-FEM Hessian: H12x3
        auto abd_fem_dytopo_effect_count =
            abd_fem_dytopo_effect_receiver->hessians().triplet_count();
        auto abd_fem_H3x3_count = abd_fem_dytopo_effect_count * 4;

        info.extent(abd_fem_H3x3_count, 0);
    }

    virtual void assemble(GlobalLinearSystem::OffDiagInfo& info) override
    {
        using namespace cuda_tool;

        auto count = abd_fem_dytopo_effect_receiver->hessians().triplet_count();

        if(count > 0)
        {
            auto k = ABDFEMLinearSubsystem_assemble_kernel;
            int  n = (int)count;
            if(n > 0)
            {
                k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                    affine_body_dynamics->v2b().viewer(),
                    affine_body_dynamics->Js().viewer(),
                    affine_body_dynamics->body_is_fixed().viewer(),
                    finite_element_method->is_fixed().viewer(),
                    info.lr_hessian().viewer(),
                    abd_fem_dytopo_effect_receiver->hessians().viewer(),
                    affine_body_vertex_reporter->vertex_offset(),
                    finite_element_vertex_reporter->vertex_offset(),
                    n);
            }
        }
    }
};

REGISTER_SIM_SYSTEM(ABDFEMLinearSubsystem);
}  // namespace uipc::backend::cuda
