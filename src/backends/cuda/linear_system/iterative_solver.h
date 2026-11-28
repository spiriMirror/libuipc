/*
 * NVIDIA contributions: fused iteration and layout-generation interfaces, and
 * the pending iteration-count interface.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

    // Iteration count of the last solve when do_solve() reported it pending
    // (SolvingInfo::iter_count_pending()); waits for that solve if needed.
    virtual SizeT do_pending_iter_count();


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
    void spmv_dot_pipelined(cuda_tool::CDenseVectorView<Float> x,
                            cuda_tool::DenseVectorView<Float>  y,
                            cuda_tool::VarView<Float>          d_dot,
                            cuda_tool::DenseVectorView<Float>  next_y,
                            cuda_tool::VarView<Float>          next_dot,
                            cuda_tool::CVarView<IndexT>        converged,
                            cudaStream_t                       stream);
    bool fused_pcg_update_apply_dot(cuda_tool::DenseVectorView<Float>  x,
                                    cuda_tool::CDenseVectorView<Float> p,
                                    cuda_tool::DenseVectorView<Float>  r,
                                    cuda_tool::CDenseVectorView<Float> Ap,
                                    cuda_tool::DenseVectorView<Float>  z,
                                    cuda_tool::CVarView<Float>         rz,
                                    cuda_tool::CVarView<Float>         pAp,
                                    cuda_tool::VarView<Float>          rz_new,
                                    cuda_tool::CVarView<IndexT> converged,
                                    cudaStream_t                stream);
    void apply_preconditioner(cuda_tool::DenseVectorView<Float>  z,
                              cuda_tool::CDenseVectorView<Float> r,
                              cuda_tool::CVarView<IndexT>        converged,
                              cudaStream_t stream = nullptr);
    // data pointers of the assembled system matrix (FusedPCG graph key)
    std::array<const void*, 3> matrix_data_ptrs() const;
    // Generation of the per-subsystem DoF layout captured by fused-PCG
    // subviews and local-preconditioner kernels.
    SizeT linear_system_layout_generation() const;
    bool  accuracy_statisfied(cuda_tool::DenseVectorView<Float> r);
    cuda_tool::LinearSystemContext& ctx() const;

  private:
    friend class GlobalLinearSystem;
    GlobalLinearSystem* m_system;

    virtual void do_build() final override;

    void  solve(GlobalLinearSystem::SolvingInfo& info);
    SizeT pending_iter_count();
};
}  // namespace uipc::backend::cuda
