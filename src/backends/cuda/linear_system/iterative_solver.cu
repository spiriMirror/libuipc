/*
 * NVIDIA contributions: fused SpMV/dot and update/apply/dot dispatch and
 * layout-generation forwarding, and the pending iteration count of an enqueued
 * solve.
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

#include <linear_system/iterative_solver.h>
#include <linear_system/global_linear_system.h>
#include <array>
namespace uipc::backend::cuda
{
void IterativeSolver::do_build()
{
    m_system = &require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_system->add_solver(this);
}

void IterativeSolver::spmv(Float                              a,
                           cuda_tool::CDenseVectorView<Float> x,
                           Float                              b,
                           cuda_tool::DenseVectorView<Float>  y)
{
    m_system->m_impl.spmv(a, x, b, y);
}

void IterativeSolver::spmv(cuda_tool::CDenseVectorView<Float> x,
                           cuda_tool::DenseVectorView<Float>  y)
{
    spmv(1.0, x, 0.0, y);
}

void IterativeSolver::spmv_dot(cuda_tool::CDenseVectorView<Float> x,
                               cuda_tool::DenseVectorView<Float>  y,
                               cuda_tool::VarView<Float>          d_dot,
                               cudaStream_t                       stream)
{
    m_system->m_impl.spmv_dot(x, y, d_dot, stream);
}

void IterativeSolver::spmv_dot_pipelined(cuda_tool::CDenseVectorView<Float> x,
                                         cuda_tool::DenseVectorView<Float>  y,
                                         cuda_tool::VarView<Float> d_dot,
                                         cuda_tool::DenseVectorView<Float> next_y,
                                         cuda_tool::VarView<Float>   next_dot,
                                         cuda_tool::CVarView<IndexT> converged,
                                         cudaStream_t                stream)
{
    m_system->m_impl.spmv_dot_pipelined(x, y, d_dot, next_y, next_dot, converged, stream);
}

bool IterativeSolver::fused_pcg_update_apply_dot(cuda_tool::DenseVectorView<Float> x,
                                                 cuda_tool::CDenseVectorView<Float> p,
                                                 cuda_tool::DenseVectorView<Float> r,
                                                 cuda_tool::CDenseVectorView<Float> Ap,
                                                 cuda_tool::DenseVectorView<Float> z,
                                                 cuda_tool::CVarView<Float> rz,
                                                 cuda_tool::CVarView<Float> pAp,
                                                 cuda_tool::VarView<Float> rz_new,
                                                 cuda_tool::CVarView<IndexT> converged,
                                                 cudaStream_t stream)
{
    return m_system->m_impl.fused_pcg_update_apply_dot(
        x, p, r, Ap, z, rz, pAp, rz_new, converged, stream);
}

void IterativeSolver::apply_preconditioner(cuda_tool::DenseVectorView<Float>  z,
                                           cuda_tool::CDenseVectorView<Float> r,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream)
{
    m_system->m_impl.apply_preconditioner(z, r, converged, stream);
}

std::array<const void*, 3> IterativeSolver::matrix_data_ptrs() const
{
    return m_system->m_impl.matrix_data_ptrs();
}

SizeT IterativeSolver::linear_system_layout_generation() const
{
    return m_system->m_impl.dof_layout_generation;
}


bool IterativeSolver::accuracy_statisfied(cuda_tool::DenseVectorView<Float> r)
{
    return m_system->m_impl.accuracy_statisfied(r);
}

cuda_tool::LinearSystemContext& IterativeSolver::ctx() const
{
    return m_system->m_impl.ctx;
}

void IterativeSolver::solve(GlobalLinearSystem::SolvingInfo& info)
{
    do_solve(info);
}

SizeT IterativeSolver::do_pending_iter_count()
{
    UIPC_ASSERT(false, "{} reported a pending iteration count without providing it", name());
    return 0;
}

SizeT IterativeSolver::pending_iter_count()
{
    return do_pending_iter_count();
}
}  // namespace uipc::backend::cuda
