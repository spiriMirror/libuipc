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

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_energy.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_function.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;

namespace
{
namespace NH = sym::neo_hookean_shell_2d;

template <typename T>
void copy_to_device(DeviceBuffer<T>& device, const std::vector<T>& host)
{
    device.resize(host.size());
    if(!host.empty())
        device.view().copy_from(host.data());
}

void check_production_energy_launcher(int count)
{
    constexpr Float Dt = Float{0.125};

    std::vector<Float>     lambdas(count);
    std::vector<Float>     mus(count);
    std::vector<Float>     rest_areas(count);
    std::vector<Float>     thicknesses(3 * count);
    std::vector<Vector3i>  indices(count);
    std::vector<Vector3>   positions(3 * count);
    std::vector<Matrix2x2> inverse_rest_shape_matrices(count);
    std::vector<Float>     expected(count);

    for(int i = 0; i < count; ++i)
    {
        const int   vertex_begin = 3 * i;
        const Float s            = Float(i % 17) * Float{1.0e-4};
        const Float thickness    = Float{0.02} + Float(i % 5) * Float{1.0e-3};

        lambdas[i]    = Float{2.0} + s;
        mus[i]        = Float{3.0} - s;
        rest_areas[i] = Float{0.5} + Float(i % 7) * Float{0.01};
        indices[i] = Vector3i{vertex_begin, vertex_begin + 1, vertex_begin + 2};
        positions[vertex_begin] = Vector3{Float{0.0}, Float{0.0}, Float{0.0}};
        positions[vertex_begin + 1] = Vector3{Float{1.0} + s, Float{0.02}, Float{0.0}};
        positions[vertex_begin + 2] = Vector3{Float{0.01}, Float{1.0} - s, Float{0.03}};
        thicknesses[vertex_begin]      = thickness;
        thicknesses[vertex_begin + 1]  = thickness;
        thicknesses[vertex_begin + 2]  = thickness;
        inverse_rest_shape_matrices[i] = Matrix2x2::Identity();

        Vector9 X;
        for(int local = 0; local < 3; ++local)
            X.segment<3>(3 * local) = positions[vertex_begin + local];
        Float energy;
        NH::E(energy, lambdas[i], mus[i], X, inverse_rest_shape_matrices[i]);
        expected[i] = energy * rest_areas[i] * Float{2.0} * thickness * Dt * Dt;
    }

    DeviceBuffer<Float>     d_lambdas;
    DeviceBuffer<Float>     d_mus;
    DeviceBuffer<Float>     d_rest_areas;
    DeviceBuffer<Float>     d_thicknesses;
    DeviceBuffer<Float>     d_energies(count);
    DeviceBuffer<Vector3i>  d_indices;
    DeviceBuffer<Vector3>   d_positions;
    DeviceBuffer<Matrix2x2> d_inverse_rest_shape_matrices;
    copy_to_device(d_lambdas, lambdas);
    copy_to_device(d_mus, mus);
    copy_to_device(d_rest_areas, rest_areas);
    copy_to_device(d_thicknesses, thicknesses);
    copy_to_device(d_indices, indices);
    copy_to_device(d_positions, positions);
    copy_to_device(d_inverse_rest_shape_matrices, inverse_rest_shape_matrices);

    launch_neo_hookean_shell_2d_energy(NeoHookeanShell2DEnergyLaunchInfo{
        .lambdas                     = d_lambdas.cview(),
        .mus                         = d_mus.cview(),
        .rest_areas                  = d_rest_areas.cview(),
        .thicknesses                 = d_thicknesses.cview(),
        .energies                    = d_energies.view(),
        .indices                     = d_indices.cview(),
        .positions                   = d_positions.cview(),
        .inverse_rest_shape_matrices = d_inverse_rest_shape_matrices.cview(),
        .dt                          = Dt});

    std::vector<Float> actual;
    d_energies.copy_to(actual);
    REQUIRE(actual.size() == expected.size());
    for(std::size_t i = 0; i < actual.size(); ++i)
    {
        const Float scale = std::max<Float>(Float{1.0}, std::abs(expected[i]));
        REQUIRE(std::isfinite(actual[i]));
        REQUIRE(std::abs(actual[i] - expected[i]) <= Float{1.0e-12} * scale);
    }
}
}  // namespace

TEST_CASE("Neo-Hookean shell production energy launcher covers CTA boundaries through 513 elements",
          "[cuda][line_search][neo_energy_launch]")
{
    for(int count : std::array{0, 1, 63, 64, 65, 127, 128, 129, 255, 256, 257, 511, 512, 513})
        check_production_energy_launcher(count);
}
