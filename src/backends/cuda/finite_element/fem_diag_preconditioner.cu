#include <type_define.h>
#include <Eigen/Dense>
#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>

namespace uipc::backend::cuda
{
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
                       {
                           return;
                       }

                       if(i == j)
                       {
                           diag_inv(i) = eigen::inverse(H3x3);
                       }
                   });
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
