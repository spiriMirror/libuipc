/*
 * NVIDIA contributions: ABD diagonal update/apply/dot fusion and its
 * local-preconditioner integration; warp-per-body diagonal block inversion.
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
#include <affine_body/abd_diag_inverse.h>
#include <linear_system/global_linear_system.h>
#include <linear_system/fused_pcg_kernels.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>
#include <utility>

namespace uipc::backend::cuda
{
namespace
{
    constexpr int      DiagInverseDim = 12;
    constexpr unsigned FullWarpMask   = 0xffffffffu;

    // The warp-cooperative cuda_tool::eigen::GaussEliminationInverse below
    // keeps column c of the augmented [H | I] matrix in col[] of lane c < 24
    // and applies exactly the operations the serial routine applies to that
    // column, in the same order. Pivot steps are template-unrolled so col[]
    // stays in registers. Lanes c < i do not take part in pivot i (the serial
    // routine starts every row loop at r = i).

    // Rows Begin..End-1 of every active column: row += e * pivot_row with
    // e = -1 * (m[j][i] / m[i][i]), both taken from column i (lane i) before
    // the update. After the row division m[i][i] = ep / ep is exactly 1 for
    // every finite nonzero ep and x / 1 == x, so such a pivot skips the
    // division; a non-finite pivot still divides. The shuffles stay outside
    // the branch on the (warp-uniform) pivot.
    template <int I, int Begin, int End>
    __device__ void abd_diag_eliminate_rows(Float (&col)[DiagInverseDim], bool active)
    {
        if constexpr(Begin < End)
        {
            const Float pivot = __shfl_sync(FullWarpMask, col[I], I);
            Float       m_ji[End - Begin];
#pragma unroll
            for(int j = Begin; j < End; j++)
                m_ji[j - Begin] = __shfl_sync(FullWarpMask, col[j], I);

            if(pivot == Float{1})
            {
#pragma unroll
                for(int j = Begin; j < End; j++)
                {
                    if(active)
                        col[j] += (-1 * m_ji[j - Begin]) * col[I];
                }
            }
            else
            {
#pragma unroll
                for(int j = Begin; j < End; j++)
                {
                    const Float e = -1 * (m_ji[j - Begin] / pivot);
                    if(active)
                        col[j] += e * col[I];
                }
            }
        }
    }

    // Forward pivot I: the serial pivot repair (add the first row j > I with
    // abs(m[j][I]) > eps when abs(m[I][I]) < eps), the row division and the
    // elimination below. Returns false where the serial routine gives up.
    template <int I>
    __device__ bool abd_diag_forward_pivot(Float (&col)[DiagInverseDim], int lane)
    {
        constexpr Float eps    = std::numeric_limits<Float>::epsilon();
        const bool      active = lane >= I && lane < 2 * DiagInverseDim;

        Float ep = __shfl_sync(FullWarpMask, col[I], I);
        if(abs(ep) < eps)
        {
            int j = DiagInverseDim;
#pragma unroll
            for(int k = DiagInverseDim - 1; k > I; k--)
            {
                if(abs(col[k]) > eps)
                    j = k;
            }
            j = __shfl_sync(FullWarpMask, j, I);
            if(j == DiagInverseDim)
                return false;
#pragma unroll
            for(int k = I + 1; k < DiagInverseDim; k++)
            {
                if(active && k == j)
                    col[I] += col[k];
            }
            ep = __shfl_sync(FullWarpMask, col[I], I);
        }
        if(active)
            col[I] /= ep;

        abd_diag_eliminate_rows<I, I + 1, DiagInverseDim>(col, active);
        return true;
    }

    template <int I>
    __device__ void abd_diag_backward_pivot(Float (&col)[DiagInverseDim], int lane)
    {
        const bool active = lane >= I && lane < 2 * DiagInverseDim;
        abd_diag_eliminate_rows<I, 0, I>(col, active);
    }

    template <int... I>
    __device__ bool abd_diag_warp_gauss_elimination(Float (&col)[DiagInverseDim],
                                                    int lane,
                                                    std::integer_sequence<int, I...>)
    {
        if(!(abd_diag_forward_pivot<I>(col, lane) && ...))
            return false;
        (abd_diag_backward_pivot<DiagInverseDim - 1 - I>(col, lane), ...);
        return true;
    }

    // One warp per body (ABDDiagInverseThreads threads per block).
    __global__ void __launch_bounds__(ABDDiagInverseThreads)
        abd_diag_preconditioner_do_assemble_kernel(cuda_tool::CBufferView<Matrix12x12> diag_hessian,
                                                   cuda_tool::BufferView<Matrix12x12> diag_inv)
    {
        const int body = blockIdx.x;
        const int lane = threadIdx.x;
        if(body >= (int)diag_hessian.size())
            return;

        constexpr int dim = DiagInverseDim;
        Float         col[dim];
#pragma unroll
        for(int k = 0; k < dim; k++)
        {
            if(lane < dim)
                col[k] = diag_hessian(body)(k, lane);
            else
                col[k] = lane - dim == k ? 1 : 0;
        }

        // Without a usable pivot the serial routine returned an uninitialized
        // matrix; store the identity instead, as the subsystem does for fixed
        // bodies.
        const bool invertible = abd_diag_warp_gauss_elimination(
            col, lane, std::make_integer_sequence<int, DiagInverseDim>{});

        if(lane >= dim && lane < 2 * dim)
        {
#pragma unroll
            for(int k = 0; k < dim; k++)
                diag_inv(body)(k, lane - dim) =
                    invertible ? col[k] : Float(lane - dim == k ? 1 : 0);
        }
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

    __global__ void abd_diag_preconditioner_fused_pcg_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_inv,
        cuda_tool::DenseVectorView<Float>   x,
        cuda_tool::CDenseVectorView<Float>  p,
        cuda_tool::DenseVectorView<Float>   r,
        cuda_tool::CDenseVectorView<Float>  Ap,
        cuda_tool::DenseVectorView<Float>   z,
        cuda_tool::CDense<Float>            rz,
        cuda_tool::CDense<Float>            pAp,
        cuda_tool::Dense<Float>             rz_new,
        cuda_tool::CDense<IndexT>           converged)
    {
        fused_pcg_block12_update_apply_dot(
            blockIdx.x, diag_inv, x, p, r, Ap, z, rz, pAp, rz_new, converged);
    }
}  // namespace

void launch_abd_diag_inverse(cuda_tool::CBufferView<Matrix12x12> diag_hessian,
                             cuda_tool::BufferView<Matrix12x12>  diag_inv,
                             cudaStream_t                        stream)
{
    UIPC_ASSERT(diag_inv.size() == diag_hessian.size(),
                "ABD diagonal inverse has {} blocks for {} Hessian blocks",
                diag_inv.size(),
                diag_hessian.size());

    const int grid_size = (int)diag_hessian.size();
    if(grid_size > 0)
    {
        abd_diag_preconditioner_do_assemble_kernel<<<grid_size, ABDDiagInverseThreads, 0, stream>>>(
            diag_hessian, diag_inv);
    }
}

void launch_fused_pcg_abd_update_apply_dot(cuda_tool::CBufferView<Matrix12x12> diag_inv,
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
    UIPC_ASSERT(x.size() == diag_inv.size() * 12,
                "ABD fused PCG segment has {} scalars for {} bodies",
                x.size(),
                diag_inv.size());

    const int grid_size = (int)diag_inv.size();
    if(grid_size > 0)
    {
        abd_diag_preconditioner_fused_pcg_kernel<<<grid_size, FusedPcgBlockDiagThreads, 0, stream>>>(
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

        launch_abd_diag_inverse(diag_hessian, diag_inv.view(), info.stream());
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

    virtual bool do_fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info) override
    {
        launch_fused_pcg_abd_update_apply_dot(diag_inv.cview(),
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
        info.block_12x12(diag_inv.cview());
        return true;
    }
};

REGISTER_SIM_SYSTEM(ABDDiagPreconditioner);
}  // namespace uipc::backend::cuda
