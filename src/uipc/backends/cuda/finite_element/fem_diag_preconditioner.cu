#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/fem_contact_receiver.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
namespace uipc::backend::cuda
{
class FEMDiagPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    FiniteElementMethod*          finite_element_method = nullptr;
    GlobalLinearSystem*           global_linear_system  = nullptr;
    FEMLinearSubsystem*           fem_linear_subsystem  = nullptr;
    muda::DeviceBuffer<Matrix3x3> diag_inv;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // This FEMDiagPreconditioner depends on FEMLinearSubsystem
        info.connect(fem_linear_subsystem);

        // after build vertex info, we can get the number of nodes
        global_vertex_manager.after_init_vertex_info(
            *this,
            [this]
            {
                auto N = finite_element_method->xs().size();
                diag_inv.resize(N);
            });
    }

    void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(diag_inv.size(),
                   [diag_hessians =
                        finite_element_method->diag_hessians().viewer().name("diag_hessians"),
                    diag_inv = diag_inv.viewer().name("diag_inv")] __device__(int i) mutable
                   {
                       // calculate the inverse of the diagonal of the hessian matrix
                       diag_inv(i) = eigen::inverse(diag_hessians(i));
                   });
    }

    void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(diag_inv.size(),
                   [diag_inv = diag_inv.viewer().name("diag_inv"),
                    r        = info.r().viewer().name("r"),
                    z = info.z().viewer().name("z")] __device__(int i) mutable
                   {
                       auto I3 = i * 3;
                       z.segment<3>(I3).as_eigen() =
                           diag_inv(i) * r.segment<3>(I3).as_eigen();
                   });
    }
};

REGISTER_SIM_SYSTEM(FEMDiagPreconditioner);
}  // namespace uipc::backend::cuda
