#include <type_define.h>
#include <Eigen/Dense>
#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/fem_contact_receiver.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>

namespace uipc::backend::cuda
{
namespace detail
{
    template <IndexT N>
    __device__ void fill_diag_hessian(muda::Dense1D<Matrix<Float, N, N>>& diagNxN,
                                      const Matrix3x3& H3x3,
                                      IndexT           local_i3,
                                      IndexT           local_j3)
    {
        constexpr IndexT M = N / 3;

        MUDA_ASSERT(local_j3 >= local_i3,
                    "dim_local_j3 should be greater than or equal to dim_local_i3, but dim_local_i3 = %d, dim_local_j3 = %d",
                    local_i3,
                    local_j3);

        MUDA_ASSERT(local_j3 - local_i3 < M,
                    "dim_local_j3 - dim_local_i3 should be less than %d, but dim_local_i3 = %d, dim_local_j3 = %d",
                    M,
                    local_i3,
                    local_j3);

        auto diagNxN_index    = local_i3 / M;
        auto diag_NxN_index_M = diagNxN_index * M;

        // get i,j offset in diag local matrix
        auto i_N = local_i3 - diag_NxN_index_M;
        auto j_N = local_j3 - diag_NxN_index_M;

        Matrix<Float, N, N>& H_NxN = diagNxN(diagNxN_index);

        // DANGER:
        // Don't use H_NxN.block<3,3>(i_N * 3, j_N * 3, 3) = H3x3,
        // the block operation write more than 3x3 elements
        // which will cause race condition.
        // so we need to write element by element

        if(i_N == j_N)  // diag
        {
            for(IndexT i = 0; i < 3; ++i)
            {
                for(IndexT j = 0; j < 3; ++j)
                {
                    muda::atomic_add(&H_NxN(i_N * 3 + i, j_N * 3 + j), H3x3(i, j));
                }
            }
        }
        else  // off-diag (consider the symmetry)
        {
            for(IndexT i = 0; i < 3; ++i)
            {
                for(IndexT j = 0; j < 3; ++j)
                {
                    muda::atomic_add(&H_NxN(i_N * 3 + i, j_N * 3 + j), H3x3(i, j));
                    muda::atomic_add(&H_NxN(j_N * 3 + j, i_N * 3 + i), H3x3(i, j));
                }
            }
        }
    }

    __device__ void fill_diag_hessian(muda::Dense1D<Matrix<Float, 3, 3>>& diagNxN,
                                      const Matrix3x3& H3x3,
                                      IndexT           dim_local_i3,
                                      IndexT           dim_local_j3)
    {
        MUDA_ASSERT(dim_local_i3 == dim_local_j3,
                    "dim_local_i3 should be equal to dim_local_j3, but dim_local_i3 = %d, dim_local_j3 = %d",
                    dim_local_i3,
                    dim_local_j3);

        diagNxN(dim_local_i3) = H3x3;
    }
}  // namespace detail


class FEMDiagPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    FiniteElementMethod* finite_element_method = nullptr;
    GlobalLinearSystem*  global_linear_system  = nullptr;
    FEMLinearSubsystem*  fem_linear_subsystem  = nullptr;

    muda::DeviceBuffer<Matrix3x3> diag_inv;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // This FEMDiagPreconditioner depends on FEMLinearSubsystem
        info.connect(fem_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        using namespace muda;

        UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Symmetric,
                    "Now only support Symmetric Hessian");

        diag_inv.resize(finite_element_method->xs().size());

        // 1) collect diagonal blocks
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.A().triplet_count(),
                   [triplet            = info.A().cviewer().name("triplet"),
                    diag_inv           = diag_inv.viewer().name("diag_inv"),
                    fem_segment_offset = info.dof_offset() / 3,
                    fem_segment_count = info.dof_count() / 3] __device__(int I) mutable
                   {
                       auto&& [g_i, g_j, H3x3] = triplet(I);

                       IndexT i = g_i - fem_segment_offset;
                       IndexT j = g_j - fem_segment_offset;

                       if(i >= fem_segment_count || j >= fem_segment_count)
                           return;

                       if(i == j)
                           detail::fill_diag_hessian(diag_inv, H3x3, i, j);
                   });

        // 2) compute inverse of diagonal blocks
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(diag_inv.size(),
                   [diag_inv = diag_inv.viewer().name("fem_3d_diag_inv")] __device__(int i) mutable
                   { diag_inv(i) = muda::eigen::inverse(diag_inv(i)); });
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(diag_inv.size(),
                   [r = info.r().viewer().name("r"),
                    z = info.z().viewer().name("z"),
                    diag_inv = diag_inv.viewer().name("diag_inv")] __device__(int i) mutable
                   {
                       z.segment<3>(i * 3).as_eigen() =
                           diag_inv(i) * r.segment<3>(i * 3).as_eigen();
                   });
    }
};

REGISTER_SIM_SYSTEM(FEMDiagPreconditioner);
}  // namespace uipc::backend::cuda
