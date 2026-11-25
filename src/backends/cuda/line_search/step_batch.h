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
#include <type_define.h>
#include <cuda_tool/buffer.h>
#include <utils/step_displacement.h>
#include <algorithm>
#include <stdexcept>

namespace uipc::backend::cuda
{
namespace details
{
    // The ranges of a StepBatch: range r covers the elements [begin[r],
    // begin[r + 1]) of the batch.
    inline constexpr int StepBatchCapacity = 8;
    enum StepBatchKind : int
    {
        StepPositions = 0,  // Vector3: x = step_position(x0, alpha, dx)
        StepDofs12    = 1,  // Vector12: q = q0 + alpha * dq, fixed bodies kept
        StepDofs3     = 2,  // Vector3: x = x0 + alpha * dx
    };
    struct StepBatchRanges
    {
        int           kind[StepBatchCapacity];
        void*         dst[StepBatchCapacity];
        const void*   start[StepBatchCapacity];
        const void*   disp[StepBatchCapacity];
        const IndexT* is_fixed[StepBatchCapacity];
        int           begin[StepBatchCapacity + 1];
        int           count;
        Float         alpha;
    };

    // One thread per element, each computing exactly what the separate
    // step_forward kernels compute for it (GlobalVertexManager,
    // ABDLineSearchReporter, FEMLineSearchReporter). A template so that only
    // the translation units that launch a batch compile it.
    template <int Capacity>
    __global__ void step_batch_kernel(StepBatchRanges r)
    {
        static_assert(Capacity == StepBatchCapacity);
        const int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= r.begin[r.count])
            return;
        int k = r.count - 1;
        while(i < r.begin[k])
            --k;
        const int j = i - r.begin[k];
        switch(r.kind[k])
        {
            case StepPositions: {
                auto*       x  = static_cast<Vector3*>(r.dst[k]);
                const auto* x0 = static_cast<const Vector3*>(r.start[k]);
                const auto* dx = static_cast<const Vector3*>(r.disp[k]);
                // Same rounding as the trajectory broad phase (see step_displacement.h).
                x[j] = step_position(x0[j], r.alpha, dx[j]);
                break;
            }
            case StepDofs12: {
                if(r.is_fixed[k][j])
                    return;
                auto*       q  = static_cast<Vector12*>(r.dst[k]);
                const auto* q0 = static_cast<const Vector12*>(r.start[k]);
                const auto* dq = static_cast<const Vector12*>(r.disp[k]);
                q[j]           = q0[j] + r.alpha * dq[j];
                break;
            }
            case StepDofs3: {
                auto*       x  = static_cast<Vector3*>(r.dst[k]);
                const auto* x0 = static_cast<const Vector3*>(r.start[k]);
                const auto* dx = static_cast<const Vector3*>(r.disp[k]);
                x[j]           = x0[j] + r.alpha * dx[j];
                break;
            }
        }
    }
}  // namespace details

// The step of a line-search trial (x = x_0 + alpha * dx) for the vertex
// positions and every reporter's DOFs, collected by positions() and dofs()
// and enqueued by launch() as one kernel instead of one kernel per system.
// Every element is computed by the same expression as in the separate
// kernels (the vertex positions through step_position(), the DOFs as
// x0 + alpha * dx), so the results are the same bytes.
class StepBatch
{
  public:
    explicit StepBatch(Float alpha) noexcept
        : m_alpha(alpha)
    {
    }
    Float alpha() const noexcept { return m_alpha; }

    // x = step_position(x0, alpha, dx): the vertex positions
    void positions(cuda_tool::BufferView<Vector3>  x,
                   cuda_tool::CBufferView<Vector3> x0,
                   cuda_tool::CBufferView<Vector3> dx)
    {
        add(details::StepPositions, x.data(), x0.data(), dx.data(), nullptr, x.size(), x0.size(), dx.size());
    }
    // q = q0 + alpha * dq for the bodies that are not fixed
    void dofs(cuda_tool::BufferView<Vector12>  q,
              cuda_tool::CBufferView<Vector12> q0,
              cuda_tool::CBufferView<Vector12> dq,
              cuda_tool::CBufferView<IndexT>   is_fixed,
              size_t                           n)
    {
        if(q.size() < n || q0.size() < n || dq.size() < n || is_fixed.size() < n)
            throw std::length_error{"StepBatch::dofs: range shorter than n"};
        add(details::StepDofs12, q.data(), q0.data(), dq.data(), is_fixed.data(), n, n, n);
    }
    // x = x0 + alpha * dx
    void dofs(cuda_tool::BufferView<Vector3>  x,
              cuda_tool::CBufferView<Vector3> x0,
              cuda_tool::CBufferView<Vector3> dx)
    {
        add(details::StepDofs3, x.data(), x0.data(), dx.data(), nullptr, x.size(), x0.size(), dx.size());
    }

    // Enqueue the collected ranges on `s` as one kernel and empty the batch.
    // (A member template: only its callers compile the kernel.)
    template <typename = void>
    void launch(cudaStream_t s = cuda_tool::default_stream())
    {
        const int n = m_ranges.begin[m_ranges.count];
        if(n > 0)
        {
            m_ranges.alpha      = m_alpha;
            constexpr int block = cuda_tool::default_block_dim;
            details::step_batch_kernel<details::StepBatchCapacity>
                <<<(n + block - 1) / block, block, 0, s>>>(m_ranges);
        }
        m_ranges = details::StepBatchRanges{};
    }

  private:
    void add(int           kind,
             void*         dst,
             const void*   start,
             const void*   disp,
             const IndexT* is_fixed,
             size_t        n,
             size_t        n_start,
             size_t        n_disp)
    {
        if(n_start != n || n_disp != n)
            throw std::runtime_error{"StepBatch: size mismatch"};
        if(n == 0)
            return;
        int k = m_ranges.count;
        if(k == details::StepBatchCapacity)
            throw std::length_error{"StepBatch: too many ranges"};
        m_ranges.kind[k]      = kind;
        m_ranges.dst[k]       = dst;
        m_ranges.start[k]     = start;
        m_ranges.disp[k]      = disp;
        m_ranges.is_fixed[k]  = is_fixed;
        m_ranges.begin[k + 1] = m_ranges.begin[k] + static_cast<int>(n);
        m_ranges.count        = k + 1;
    }

    Float                    m_alpha;
    details::StepBatchRanges m_ranges{};
};
}  // namespace uipc::backend::cuda
