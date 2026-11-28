/*
 * NVIDIA contributions: FEM diagonal update/apply/dot fusion and its
 * local-preconditioner integration, including the block-diagonal inverse
 * served to the combined ABD/FEM step.
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

#include <type_define.h>
#include <Eigen/Dense>
#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <linear_system/global_linear_system.h>
#include <linear_system/fused_pcg_kernels.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <uipc/geometry/simplicial_complex.h>
#include <cub/block/block_reduce.cuh>

namespace uipc::backend::cuda
{
namespace
{
    namespace eigen = cuda_tool::eigen;

    __global__ void FEMDiagPreconditioner_do_assemble_kernel(
        cuda_tool::CBCOOMatrixView<Float, 3> triplet,
        cuda_tool::BufferView<Matrix3x3>     diag_inv,
        SizeT                                fem_segment_offset,
        SizeT                                fem_segment_count,
        int                                  n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [g_i, g_j, H3x3] = triplet(I);

        IndexT i = g_i - fem_segment_offset;
        IndexT j = g_j - fem_segment_offset;

        if(i >= fem_segment_count || j >= fem_segment_count)
        {
            return;
        }

        if(i == j)
        {
            diag_inv(i) = eigen::inverse(H3x3);
        }
    }

    __global__ void FEMDiagPreconditioner_do_apply_kernel(
        cuda_tool::CDenseVectorView<Float> r,
        cuda_tool::DenseVectorView<Float>  z,
        cuda_tool::CDense<IndexT>          converged,
        cuda_tool::BufferView<Matrix3x3>   diag_inv,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*converged != 0)
            return;
        z.segment<3>(i * 3).as_eigen() = diag_inv(i) * r.segment<3>(i * 3).as_eigen();
    }

    __global__ void fem_diag_preconditioner_fused_pcg_kernel(
        cuda_tool::CBufferView<Matrix3x3>  diag_inv,
        cuda_tool::DenseVectorView<Float>  x,
        cuda_tool::CDenseVectorView<Float> p,
        cuda_tool::DenseVectorView<Float>  r,
        cuda_tool::CDenseVectorView<Float> Ap,
        cuda_tool::DenseVectorView<Float>  z,
        cuda_tool::CDense<Float>           rz,
        cuda_tool::CDense<Float>           pAp,
        cuda_tool::Dense<Float>            rz_new,
        cuda_tool::CDense<IndexT>          converged)
    {
        fused_pcg_block3_update_apply_dot(
            blockIdx.x, diag_inv, x, p, r, Ap, z, rz, pAp, rz_new, converged);
    }
}  // namespace

void launch_fused_pcg_fem_update_apply_dot(cuda_tool::CBufferView<Matrix3x3> diag_inv,
                                           cuda_tool::DenseVectorView<Float>  x,
                                           cuda_tool::CDenseVectorView<Float> p,
                                           cuda_tool::DenseVectorView<Float>  r,
                                           cuda_tool::CDenseVectorView<Float> Ap,
                                           cuda_tool::DenseVectorView<Float> z,
                                           cuda_tool::CVarView<Float>        rz,
                                           cuda_tool::CVarView<Float> pAp,
                                           cuda_tool::VarView<Float>  rz_new,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream)
{
    UIPC_ASSERT(x.size() == diag_inv.size() * 3,
                "FEM fused PCG segment has {} scalars for {} vertices",
                x.size(),
                diag_inv.size());

    const int vertex_count = (int)diag_inv.size();
    const int grid_size = (vertex_count + FusedPcgBlockDiagThreads - 1) / FusedPcgBlockDiagThreads;
    if(grid_size > 0)
    {
        fem_diag_preconditioner_fused_pcg_kernel<<<grid_size, FusedPcgBlockDiagThreads, 0, stream>>>(
            diag_inv,
            x,
            p,
            r,
            Ap,
            z,
            rz.cviewer(),
            pAp.cviewer(),
            rz_new.viewer(),
            converged.cviewer());
    }
}

class FEMDiagPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    FiniteElementMethod* finite_element_method = nullptr;
    GlobalLinearSystem*  global_linear_system  = nullptr;
    FEMLinearSubsystem*  fem_linear_subsystem  = nullptr;

    cuda_tool::DeviceBuffer<Matrix3x3> diag_inv;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // MAS is selected via config; defer to FEMMASPreconditioner when on.
        auto precond = world().scene().config().find<std::string>("linear_system/fem_preconditioner");
        if(precond && precond->view()[0] == "mas")
        {
            throw SimSystemException(
                "FEMDiagPreconditioner: linear_system/fem_preconditioner == \"mas\", "
                "deferring to FEMMASPreconditioner.");
        }

        // This FEMDiagPreconditioner depends on FEMLinearSubsystem
        info.connect(fem_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        diag_inv.resize(finite_element_method->xs().size());

        // 1) collect diagonal blocks
        auto k = FEMDiagPreconditioner_do_assemble_kernel;
        int  n = (int)info.A().triplet_count();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.A(), diag_inv.view(), info.dof_offset() / 3, info.dof_count() / 3, n);
        }
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        auto converged = info.converged();

        auto k = FEMDiagPreconditioner_do_apply_kernel;
        int  n = (int)diag_inv.size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream()>>>(
                info.r(), info.z(), converged.cviewer(), diag_inv.view(), n);
        }
    }

    virtual bool do_fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info) override
    {
        launch_fused_pcg_fem_update_apply_dot(diag_inv.cview(),
                                              info.x(),
                                              info.p(),
                                              info.r(),
                                              info.Ap(),
                                              info.z(),
                                              info.rz(),
                                              info.pAp(),
                                              info.rz_new(),
                                              info.converged(),
                                              info.stream());
        return true;
    }

    virtual bool do_fused_pcg_diag_inverse(GlobalLinearSystem::FusedPcgDiagInverseInfo& info) override
    {
        info.block_3x3(diag_inv.cview());
        return true;
    }
};

REGISTER_SIM_SYSTEM(FEMDiagPreconditioner);
}  // namespace uipc::backend::cuda
