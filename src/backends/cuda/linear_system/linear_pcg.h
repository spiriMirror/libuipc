#pragma once
#include <linear_system/iterative_solver.h>

namespace uipc::backend::cuda
{
class LinearPCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;


  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector = muda::DeviceDenseVector<Float>;
    using DeviceBCOOMatrix  = muda::DeviceBCOOMatrix<Float, 3>;
    using DeviceBSRMatrix   = muda::DeviceBSRMatrix<Float, 3>;

    SizeT pcg(muda::DenseVectorView<Float> x, muda::CDenseVectorView<Float> b, SizeT max_iter);

    DeviceDenseVector      z;   // preconditioned residual
    DeviceDenseVector      r;   // residual
    DeviceDenseVector      p;   // search direction
    DeviceDenseVector      Ap;  // A*p
    muda::DeviceVar<Float> dot_res;

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
};
}  // namespace uipc::backend::cuda
