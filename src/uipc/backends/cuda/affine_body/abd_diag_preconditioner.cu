#include <affine_body/abd_diag_preconditioner.h>
#include <affine_body/abd_linear_subsystem.h>
#include <linear_system/global_linear_system.h>
#include <muda/ext/eigen/inverse.h>
#include <kernel_cout.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDDiagPreconditioner);

void ABDDiagPreconditioner::do_build()
{
    auto& global_linear_system  = require<GlobalLinearSystem>();
    auto  abd_linear_subsystem  = &require<ABDLinearSubsystem>();
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();

    global_linear_system.add_preconditioner(this, abd_linear_subsystem);
}

void ABDDiagPreconditioner::Impl::assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info)
{
    using namespace muda;

    inv_diag.resize(abd().diag_hessian.size());

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(inv_diag.size(),
               [diag_hessian = abd().diag_hessian.viewer().name("diag_hessian"),
                inv_diag = inv_diag.viewer().name("inv_diag")] __device__(int i) mutable
               { inv_diag(i) = muda::eigen::inverse(diag_hessian(i)); });
}

void ABDDiagPreconditioner::Impl::apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    using namespace muda;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(inv_diag.size(),
               [r = info.r().viewer().name("r"),
                z = info.z().viewer().name("z"),
                inv_diag = inv_diag.viewer().name("inv_diag")] __device__(int i) mutable
               {
                   z.segment<12>(i * 12).as_eigen() =
                       inv_diag(i) * r.segment<12>(i * 12).as_eigen();
               });
}

void ABDDiagPreconditioner::do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info)
{
    m_impl.assemble(info);
}

void ABDDiagPreconditioner::do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    m_impl.apply(info);
}
}  // namespace uipc::backend::cuda
