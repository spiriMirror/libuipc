/*
 * NVIDIA contributions: preconditioner assembly on the provided stream.
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

#include <linear_system/local_preconditioner.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_linear_subsystem.h>
#include <linear_system/global_linear_system.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_diag_preconditioner_do_assemble_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_hessian,
        cuda_tool::BufferView<Matrix12x12>  diag_inv,
        int                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        diag_inv(i) = cuda_tool::eigen::inverse(diag_hessian(i));
    }

    __global__ void abd_diag_preconditioner_do_apply_kernel(
        cuda_tool::CDenseVectorView<Float> r,
        cuda_tool::DenseVectorView<Float>  z,
        cuda_tool::CDense<IndexT>          converged,
        cuda_tool::BufferView<Matrix12x12> diag_inv,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*converged != 0)
            return;
        z.segment<12>(i * 12).as_eigen() = diag_inv(i) * r.segment<12>(i * 12).as_eigen();
    }
}  // namespace

class ABDDiagPreconditioner final : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    ABDLinearSubsystem* abd_linear_subsystem = nullptr;

    cuda_tool::DeviceBuffer<Matrix12x12> diag_inv;

    virtual void do_build(BuildInfo& info) override
    {
        auto& global_linear_system = require<GlobalLinearSystem>();
        abd_linear_subsystem       = &require<ABDLinearSubsystem>();

        info.connect(abd_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        // All device work stays on info.stream(), which may be a side stream.
        auto diag_hessian = abd_linear_subsystem->diag_hessian();
        diag_inv.resize(diag_hessian.size(), info.stream());

        int n = (int)diag_inv.size();
        if(n > 0)
        {
            auto k = abd_diag_preconditioner_do_assemble_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream()>>>(
                diag_hessian, diag_inv.view(), n);
        }
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        auto converged = info.converged();

        int n = (int)diag_inv.size();
        if(n > 0)
        {
            auto k = abd_diag_preconditioner_do_apply_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream()>>>(
                info.r(), info.z(), converged.cviewer(), diag_inv.view(), n);
        }
    }
};

REGISTER_SIM_SYSTEM(ABDDiagPreconditioner);
}  // namespace uipc::backend::cuda
