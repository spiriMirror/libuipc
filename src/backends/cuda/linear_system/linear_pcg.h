#pragma once
#include <linear_system/iterative_solver.h>
#include <cuda_tool/cuda_tool.h>

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
    using DeviceDenseVector = cuda_tool::DeviceDenseVector<Float>;
    using DeviceBCOOMatrix  = cuda_tool::DeviceBCOOMatrix<Float, 3>;
    using DeviceBSRMatrix   = cuda_tool::DeviceBSRMatrix<Float, 3>;

    SizeT pcg(cuda_tool::DenseVectorView<Float>  x,
              cuda_tool::CDenseVectorView<Float> b,
              SizeT                              max_iter);
    void  dump_r_z(SizeT k);
    void  dump_p_Ap(SizeT k);
    void  check_init_rz_nan_inf(Float rz);
    void  check_iter_rz_nan_inf(Float rz, SizeT k);

    DeviceDenseVector            r0;  // initial residual
    DeviceDenseVector            z;   // preconditioned residual
    DeviceDenseVector            r;   // residual
    DeviceDenseVector            p;   // search direction
    DeviceDenseVector            Ap;  // A*p
    cuda_tool::DeviceVar<IndexT> d_converged_false;

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;

    bool        need_debug_dump = false;
    std::string debug_dump_path;
};
}  // namespace uipc::backend::cuda
