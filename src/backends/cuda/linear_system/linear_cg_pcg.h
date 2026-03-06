#pragma once
#include <linear_system/iterative_solver.h>

namespace uipc::backend::cuda
{
// Chronopoulos-Gear PCG:
// Fuses the two dot products (p^T*Ap and r^T*z) per iteration into a
// single 2-element reduction, halving the number of global synchronizations.
// Reference: Chronopoulos & Gear, "s-step Iterative Methods for Symmetric
// Linear Systems," J. Comp. Appl. Math., 1989.
class LinearCGPCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector = muda::DeviceDenseVector<Float>;

    SizeT cg_pcg(muda::DenseVectorView<Float> x,
                 muda::CDenseVectorView<Float> b,
                 SizeT                         max_iter);

    // Buffers
    DeviceDenseVector r;   // residual
    DeviceDenseVector z;   // preconditioned residual  M^{-1} r
    DeviceDenseVector p;   // search direction
    DeviceDenseVector Ap;  // A * p

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
};
}  // namespace uipc::backend::cuda
