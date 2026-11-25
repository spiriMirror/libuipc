/*
 * NVIDIA contributions: stream plumbing and direct gradients of the
 * prelaunched FEM assembly and the line-search energy.
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

#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
void FEM3DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    FEM3DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void FEM3DConstitution::do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info)
{
    FEM3DConstitution::ComputeEnergyInfo this_info{
        this, m_index_in_dim, info.dt(), info.energies(), &info};
    do_compute_energy(this_info);
}

void FEM3DConstitution::do_compute_gradient_hessian(FiniteElementConstitution::ComputeGradientHessianInfo& info)
{
    FEM3DConstitution::ComputeGradientHessianInfo this_info{this,
                                                            m_index_in_dim,
                                                            info.gradient_only(),
                                                            info.dt(),
                                                            info.gradients(),
                                                            info.hessians(),
                                                            info.stream()};
    this_info.m_base = &info;
    do_compute_gradient_hessian(this_info);
}

IndexT FEM3DConstitution::get_dim() const noexcept
{
    return 3;
}

cuda_tool::CBufferView<Vector4i> FEM3DConstitution::BaseInfo::indices() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().tets.view(info.primitive_offset, info.primitive_count);
}

cuda_tool::CBufferView<Vector3> FEM3DConstitution::BaseInfo::xs() const noexcept
{
    return m_impl->fem().xs.view();  // must return full buffer, because the indices index into the full buffer
}

cuda_tool::CBufferView<Vector3> FEM3DConstitution::BaseInfo::x_bars() const noexcept
{
    return m_impl->fem().x_bars.view();  // must return full buffer, because the indices index into the full buffer
}

cuda_tool::CBufferView<Matrix3x3> FEM3DConstitution::BaseInfo::Dm_invs() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().Dm3x3_invs.view(info.primitive_offset, info.primitive_count);
}

cuda_tool::CBufferView<Float> FEM3DConstitution::BaseInfo::rest_volumes() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().rest_volumes.view(info.primitive_offset, info.primitive_count);
}

const FiniteElementMethod::ConstitutionInfo& FEM3DConstitution::BaseInfo::constitution_info() const noexcept
{
    return m_impl->constitution_info();
}
}  // namespace uipc::backend::cuda
