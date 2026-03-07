#pragma once
#include <linear_system/iterative_solver.h>
#include <muda/buffer/device_var.h>

namespace uipc::backend::cuda
{
// Fused PCG: keeps dot-product scalars (rz, pAp, rz_new) on device
// to eliminate per-iteration host synchronizations.  The update kernels read
// alpha = rz/pAp and beta = rz_new/rz directly from device memory.
// SpMV and dot(p,Ap) are fused into a single kernel pass.
// Convergence is checked every `check_interval` iterations via a single D2H copy.
class LinearFusedPCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector = muda::DeviceDenseVector<Float>;

    SizeT fused_pcg(muda::DenseVectorView<Float>  x,
                    muda::CDenseVectorView<Float> b,
                    SizeT                         max_iter);

    DeviceDenseVector r;
    DeviceDenseVector z;
    DeviceDenseVector p;
    DeviceDenseVector Ap;

    muda::DeviceVar<Float> d_rz;
    muda::DeviceVar<Float> d_pAp;
    muda::DeviceVar<Float> d_rz_new;
    muda::DeviceVar<int>   d_converged;

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
    SizeT check_interval  = 5;
};
}  // namespace uipc::backend::cuda
