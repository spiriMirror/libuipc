#include <linear_system/iterative_solver.h>
#include <linear_system/global_linear_system.h>
#include <array>
namespace uipc::backend::cuda
{
void IterativeSolver::do_build()
{
    m_system = &require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_system->add_solver(this);
}

void IterativeSolver::spmv(Float                              a,
                           cuda_tool::CDenseVectorView<Float> x,
                           Float                              b,
                           cuda_tool::DenseVectorView<Float>  y)
{
    m_system->m_impl.spmv(a, x, b, y);
}

void IterativeSolver::spmv(cuda_tool::CDenseVectorView<Float> x,
                           cuda_tool::DenseVectorView<Float>  y)
{
    spmv(1.0, x, 0.0, y);
}

void IterativeSolver::spmv_dot(cuda_tool::CDenseVectorView<Float> x,
                               cuda_tool::DenseVectorView<Float>  y,
                               cuda_tool::VarView<Float>          d_dot,
                               cudaStream_t                       stream)
{
    m_system->m_impl.spmv_dot(x, y, d_dot, stream);
}

void IterativeSolver::apply_preconditioner(cuda_tool::DenseVectorView<Float>  z,
                                           cuda_tool::CDenseVectorView<Float> r,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream)
{
    m_system->m_impl.apply_preconditioner(z, r, converged, stream);
}

std::array<const void*, 3> IterativeSolver::matrix_data_ptrs() const
{
    return m_system->m_impl.matrix_data_ptrs();
}

SizeT IterativeSolver::matrix_triplet_count() const
{
    return m_system->m_impl.matrix_triplet_count();
}

bool IterativeSolver::accuracy_statisfied(cuda_tool::DenseVectorView<Float> r)
{
    return m_system->m_impl.accuracy_statisfied(r);
}

cuda_tool::LinearSystemContext& IterativeSolver::ctx() const
{
    return m_system->m_impl.ctx;
}

void IterativeSolver::solve(GlobalLinearSystem::SolvingInfo& info)
{
    do_solve(info);
}
}  // namespace uipc::backend::cuda
