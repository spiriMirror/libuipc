#pragma once
#include <sim_system.h>
#include <cuda_tool/cuda_tool.h>
#include <linear_system/global_linear_system.h>

namespace uipc::backend::cuda
{
class IterativeSolver : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;

    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) = 0;


    /**********************************************************************************************
    * Util functions for derived classes
    ***********************************************************************************************/

    void spmv(Float                              a,
              cuda_tool::CDenseVectorView<Float> x,
              Float                              b,
              cuda_tool::DenseVectorView<Float>  y);
    void spmv(cuda_tool::CDenseVectorView<Float> x, cuda_tool::DenseVectorView<Float> y);
    void spmv_dot(cuda_tool::CDenseVectorView<Float> x,
                  cuda_tool::DenseVectorView<Float>  y,
                  cuda_tool::VarView<Float>          d_dot,
                  cudaStream_t                       stream = nullptr);
    void apply_preconditioner(cuda_tool::DenseVectorView<Float>  z,
                              cuda_tool::CDenseVectorView<Float> r,
                              cuda_tool::CVarView<IndexT>        converged,
                              cudaStream_t stream = nullptr);
    // data pointers of the assembled system matrix (FusedPCG graph key)
    std::array<const void*, 3> matrix_data_ptrs() const;
    // baked into the captured SpMV kernel; part of the graph key
    SizeT matrix_triplet_count() const;
    bool accuracy_statisfied(cuda_tool::DenseVectorView<Float> r);
    cuda_tool::LinearSystemContext& ctx() const;

  private:
    friend class GlobalLinearSystem;
    GlobalLinearSystem* m_system;

    virtual void do_build() final override;

    void solve(GlobalLinearSystem::SolvingInfo& info);
};
}  // namespace uipc::backend::cuda
