/*
 * NVIDIA contributions: stream plumbing of the prelaunched FEM assembly.
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

#include <finite_element/codim_2d_constitution.h>

namespace uipc::backend::cuda
{
void Codim2DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim2DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void Codim2DConstitution::do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info)
{
    Codim2DConstitution::ComputeEnergyInfo this_info{
        this, m_index_in_dim, info.dt(), info.energies()};
    do_compute_energy(this_info);
}

void Codim2DConstitution::do_compute_gradient_hessian(FiniteElementConstitution::ComputeGradientHessianInfo& info)
{
    Codim2DConstitution::ComputeGradientHessianInfo this_info{this,
                                                              m_index_in_dim,
                                                              info.gradient_only(),
                                                              info.dt(),
                                                              info.gradients(),
                                                              info.hessians(),
                                                              info.stream()};
    do_compute_gradient_hessian(this_info);
}

const FiniteElementMethod::ConstitutionInfo& Codim2DConstitution::constitution_info() const noexcept
{
    return fem().codim_2d_constitution_infos[m_index_in_dim];
}

IndexT Codim2DConstitution::get_dim() const noexcept
{
    return 2;
}

cuda_tool::CBufferView<Vector3> Codim2DConstitution::BaseInfo::xs() const noexcept
{
    return m_impl->fem().xs.view();  // must return full buffer, because the indices index into the full buffer
}

cuda_tool::CBufferView<Vector3> Codim2DConstitution::BaseInfo::x_bars() const noexcept
{
    return m_impl->fem().x_bars.view();  // must return full buffer, because the indices index into the full buffer
}

cuda_tool::CBufferView<Float> Codim2DConstitution::BaseInfo::rest_areas() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().rest_areas.view(info.primitive_offset, info.primitive_count);
}

cuda_tool::CBufferView<Float> Codim2DConstitution::BaseInfo::thicknesses() const noexcept
{
    return m_impl->fem().thicknesses.view();
}

cuda_tool::CBufferView<Vector3i> Codim2DConstitution::BaseInfo::indices() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().codim_2ds.view(info.primitive_offset, info.primitive_count);
}

const FiniteElementMethod::ConstitutionInfo& Codim2DConstitution::BaseInfo::constitution_info() const noexcept
{
    return m_impl->constitution_info();
}

Float Codim2DConstitution::BaseInfo::dt() const noexcept
{
    return m_dt;
}
}  // namespace uipc::backend::cuda
