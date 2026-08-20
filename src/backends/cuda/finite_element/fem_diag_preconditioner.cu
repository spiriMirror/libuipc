#include <type_define.h>
#include <Eigen/Dense>
#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace eigen = cuda_tool::eigen;

    __global__ void FEMDiagPreconditioner_do_assemble_kernel(
        cuda_tool::CBCOOMatrixView<Float, 3> triplet,
        cuda_tool::BufferView<Matrix3x3>     diag_inv,
        SizeT                                fem_segment_offset,
        SizeT                                fem_segment_count,
        int                                  n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [g_i, g_j, H3x3] = triplet(I);

        IndexT i = g_i - fem_segment_offset;
        IndexT j = g_j - fem_segment_offset;

        if(i >= fem_segment_count || j >= fem_segment_count)
        {
            return;
        }

        if(i == j)
        {
            diag_inv(i) = eigen::inverse(H3x3);
        }
    }

    __global__ void FEMDiagPreconditioner_do_apply_kernel(
        cuda_tool::CDenseVectorView<Float> r,
        cuda_tool::DenseVectorView<Float>  z,
        cuda_tool::CDense<IndexT>          converged,
        cuda_tool::BufferView<Matrix3x3>   diag_inv,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*converged != 0)
            return;
        z.segment<3>(i * 3).as_eigen() =
            diag_inv(i) * r.segment<3>(i * 3).as_eigen();
    }
}  // namespace

class FEMDiagPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    FiniteElementMethod* finite_element_method = nullptr;
    GlobalLinearSystem*  global_linear_system  = nullptr;
    FEMLinearSubsystem*  fem_linear_subsystem  = nullptr;

    cuda_tool::DeviceBuffer<Matrix3x3> diag_inv;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // If ANY FEM geometry has mesh_part, defer to FEMMASPreconditioner,
        // which handles both partitioned (MAS) and unpartitioned (diag) vertices.
        auto geo_slots = world().scene().geometries();
        for(SizeT i = 0; i < geo_slots.size(); i++)
        {
            auto& geo = geo_slots[i]->geometry();
            auto* sc  = geo.as<geometry::SimplicialComplex>();
            if(sc && sc->dim() >= 1)
            {
                auto mesh_part = sc->vertices().find<IndexT>("mesh_part");
                if(mesh_part)
                {
                    throw SimSystemException(
                        "FEMDiagPreconditioner: mesh_part found, "
                        "deferring to FEMMASPreconditioner.");
                }
            }
        }

        // This FEMDiagPreconditioner depends on FEMLinearSubsystem
        info.connect(fem_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        diag_inv.resize(finite_element_method->xs().size());

        // 1) collect diagonal blocks
        auto k = FEMDiagPreconditioner_do_assemble_kernel;
        int  n = (int)info.A().triplet_count();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.A(),
                diag_inv.view(),
                info.dof_offset() / 3,
                info.dof_count() / 3,
                n);
        }
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        auto converged = info.converged();

        auto k = FEMDiagPreconditioner_do_apply_kernel;
        int  n = (int)diag_inv.size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.r(), info.z(), converged.cviewer(), diag_inv.view(), n);
        }
    }
};

REGISTER_SIM_SYSTEM(FEMDiagPreconditioner);
}  // namespace uipc::backend::cuda
