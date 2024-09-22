#include <linear_system/iterative_solver.h>
#include <linear_system/global_linear_system.h>
namespace uipc::backend::cuda
{
void IterativeSolver::do_build()
{
    m_system = &require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_system->add_solver(this);
}

void IterativeSolver::spmv(Float                         a,
                           muda::CDenseVectorView<Float> x,
                           Float                         b,
                           muda::DenseVectorView<Float>  y)
{
    m_system->m_impl.spmv(a, x, b, y);
}

void IterativeSolver::spmv(muda::CDenseVectorView<Float> x, muda::DenseVectorView<Float> y)
{
    spmv(1.0, x, 0.0, y);
}

void IterativeSolver::apply_preconditioner(muda::DenseVectorView<Float>  z,
                                           muda::CDenseVectorView<Float> r)
{
    m_system->m_impl.apply_preconditioner(z, r);
}

bool IterativeSolver::accuracy_statisfied(muda::DenseVectorView<Float> r)
{
    return m_system->m_impl.accuracy_statisfied(r);
}

muda::LinearSystemContext& IterativeSolver::ctx() const
{
    return m_system->m_impl.ctx;
}

void IterativeSolver::solve(GlobalLinearSystem::SolvingInfo& info)
{
    do_solve(info);
}
}  // namespace uipc::backend::cuda
