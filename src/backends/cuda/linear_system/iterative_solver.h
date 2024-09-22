#pragma once
#include <sim_system.h>
#include <muda/ext/linear_system.h>
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

    void spmv(Float a, muda::CDenseVectorView<Float> x, Float b, muda::DenseVectorView<Float> y);
    void spmv(muda::CDenseVectorView<Float> x, muda::DenseVectorView<Float> y);
    void apply_preconditioner(muda::DenseVectorView<Float>  z,
                              muda::CDenseVectorView<Float> r);
    bool accuracy_statisfied(muda::DenseVectorView<Float> r);
    muda::LinearSystemContext& ctx() const;

  private:
    friend class GlobalLinearSystem;
    GlobalLinearSystem* m_system;

    virtual void do_build() final override;

    void solve(GlobalLinearSystem::SolvingInfo& info);
};
}  // namespace uipc::backend::cuda
