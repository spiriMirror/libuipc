#pragma once
#include <linear_system/iterative_solver.h>

namespace uipc::backend::cuda
{
// Pipelined PCG (Ghysels & Vanroose, 2014):
// Reorganizes recurrences so that SpMV and preconditioning of the *next*
// iterate are overlapped with the current iteration's single global reduce,
// reducing synchronization overhead.
// Reference: Ghysels & Vanroose, "Hiding global synchronization latency in
// the preconditioned conjugate gradient algorithm," Parallel Computing, 2014.
class LinearPipePCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector = muda::DeviceDenseVector<Float>;

    SizeT pipe_pcg(muda::DenseVectorView<Float> x,
                   muda::CDenseVectorView<Float> b,
                   SizeT                          max_iter);

    // Standard PCG buffers
    DeviceDenseVector r;   // residual
    DeviceDenseVector z;   // M^{-1} r
    DeviceDenseVector p;   // search direction
    DeviceDenseVector Ap;  // A * p
    // Extra pipelined buffer
    DeviceDenseVector w;   // A * z  (look-ahead SpMV)

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
};
}  // namespace uipc::backend::cuda
