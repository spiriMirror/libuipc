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
#include <contact_system/contact_models/ipc_simplex_normal_contact_assembly.h>
#include <cuda_tool/cuda_tool.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;

namespace
{
constexpr IndexT IndexCanary = -123456;
constexpr Float  ValueCanary = 9876.25;

struct Counts
{
    int pt = 0;
    int ee = 0;
    int pe = 0;
    int pp = 0;
};

template <typename T>
void copy_prefix(DeviceBuffer<T>& device, const std::vector<T>& host, int count)
{
    device.resize(count);
    if(count > 0)
        device.view().copy_from(host.data());
}

struct Outputs
{
    DeviceDoubletVector<Float, 3> full_gradient;
    DeviceDoubletVector<Float, 3> only_gradient;
    DeviceTripletMatrix<Float, 3> full_hessian;
    DeviceTripletMatrix<Float, 3> untouched_hessian;
    int                           gradient_count = 0;
    int                           hessian_count  = 0;

    static void reset_gradient(DeviceDoubletVector<Float, 3>& buffer)
    {
        buffer.indices().fill(IndexCanary);
        const Vector3 canary = Vector3::Constant(ValueCanary);
        buffer.values().fill(canary);
    }

    static void reset_hessian(DeviceTripletMatrix<Float, 3>& buffer)
    {
        buffer.row_indices().fill(IndexCanary);
        buffer.col_indices().fill(IndexCanary);
        const Matrix3x3 canary = Matrix3x3::Constant(ValueCanary);
        buffer.values().fill(canary);
    }

    void prepare(int vertices, int count, int arity)
    {
        gradient_count = count * arity;
        hessian_count  = count * arity * (arity + 1) / 2;
        // Offset subviews leave an independently checked guard on both sides,
        // including when an individual contact type is empty.
        full_gradient.resize(vertices, gradient_count + 2);
        only_gradient.resize(vertices, gradient_count + 2);
        full_hessian.resize(vertices, vertices, hessian_count + 2);
        untouched_hessian.resize(vertices, vertices, hessian_count + 2);
        reset_gradient(full_gradient);
        reset_gradient(only_gradient);
        reset_hessian(full_hessian);
        reset_hessian(untouched_hessian);
    }

    auto gradient_view(bool gradient_only)
    {
        auto& buffer = gradient_only ? only_gradient : full_gradient;
        return buffer.view().subview(1, gradient_count);
    }

    auto hessian_view(bool gradient_only)
    {
        auto& buffer = gradient_only ? untouched_hessian : full_hessian;
        return buffer.view().subview(1, hessian_count);
    }
};

template <int N>
void check_outputs(Outputs& outputs, const std::vector<Vector<IndexT, N>>& contacts, int count)
{
    std::vector<IndexT>  full_indices(outputs.gradient_count + 2);
    std::vector<IndexT>  only_indices(outputs.gradient_count + 2);
    std::vector<Vector3> full_values(outputs.gradient_count + 2);
    std::vector<Vector3> only_values(outputs.gradient_count + 2);
    outputs.full_gradient.indices().copy_to(full_indices.data());
    outputs.only_gradient.indices().copy_to(only_indices.data());
    outputs.full_gradient.values().copy_to(full_values.data());
    outputs.only_gradient.values().copy_to(only_values.data());

    for(int guard : {0, outputs.gradient_count + 1})
    {
        REQUIRE(full_indices[guard] == IndexCanary);
        REQUIRE(only_indices[guard] == IndexCanary);
        REQUIRE((full_values[guard].array() == ValueCanary).all());
        REQUIRE((only_values[guard].array() == ValueCanary).all());
    }
    for(int i = 0; i < count; ++i)
    {
        Float squared_norm = 0.0;
        for(int local = 0; local < N; ++local)
        {
            const int slot = 1 + i * N + local;
            REQUIRE(full_indices[slot] == contacts[i](local));
            REQUIRE(only_indices[slot] == contacts[i](local));
            for(int axis = 0; axis < 3; ++axis)
            {
                const Float full = full_values[slot](axis);
                const Float only = only_values[slot](axis);
                REQUIRE(std::isfinite(full));
                REQUIRE(std::isfinite(only));
                REQUIRE(full != ValueCanary);
                REQUIRE(only != ValueCanary);
                const Float scale =
                    std::max({Float{1.0}, std::abs(full), std::abs(only)});
                REQUIRE(std::abs(full - only) <= Float{1.0e-10} * scale);
                squared_norm += full * full;
            }
        }
        // The fixture is inside the active barrier, so a no-op/zero-gradient
        // implementation cannot make the two production variants agree.
        REQUIRE(squared_norm > 0.0);
    }

    const int           hessian_slots = outputs.hessian_count + 2;
    std::vector<IndexT> full_rows(hessian_slots), full_cols(hessian_slots);
    std::vector<IndexT> only_rows(hessian_slots), only_cols(hessian_slots);
    std::vector<Matrix3x3> full_blocks(hessian_slots), only_blocks(hessian_slots);
    outputs.full_hessian.row_indices().copy_to(full_rows.data());
    outputs.full_hessian.col_indices().copy_to(full_cols.data());
    outputs.full_hessian.values().copy_to(full_blocks.data());
    outputs.untouched_hessian.row_indices().copy_to(only_rows.data());
    outputs.untouched_hessian.col_indices().copy_to(only_cols.data());
    outputs.untouched_hessian.values().copy_to(only_blocks.data());
    for(int slot = 0; slot < hessian_slots; ++slot)
    {
        REQUIRE(only_rows[slot] == IndexCanary);
        REQUIRE(only_cols[slot] == IndexCanary);
        REQUIRE((only_blocks[slot].array() == ValueCanary).all());
    }
    for(int guard : {0, hessian_slots - 1})
    {
        REQUIRE(full_rows[guard] == IndexCanary);
        REQUIRE(full_cols[guard] == IndexCanary);
        REQUIRE((full_blocks[guard].array() == ValueCanary).all());
    }
    int slot = 1;
    for(int i = 0; i < count; ++i)
        for(int row = 0; row < N; ++row)
            for(int col = row; col < N; ++col, ++slot)
            {
                REQUIRE(full_rows[slot] == std::min(contacts[i](row), contacts[i](col)));
                REQUIRE(full_cols[slot] == std::max(contacts[i](row), contacts[i](col)));
                REQUIRE(full_blocks[slot].allFinite());
                REQUIRE((full_blocks[slot].array() != ValueCanary).all());
            }
}

class AssemblyFixture
{
  public:
    static constexpr int MaxPerType = 257;

    AssemblyFixture()
        : contact_tabular(Extent2D{1, 1})
    {
        ContactCoeff coeff;
        coeff.kappa = 3.25;
        coeff.mu    = 0.0;
        contact_tabular.view().copy_from(&coeff);

        // Reuse the nondegenerate PT/intersecting-projection EE/PE/PP geometry
        // from the production energy-launch tests. Distances stay in (0, d_hat)
        // even for the larger launch-boundary cases.
        std::vector<Vector3> positions;
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{3.0} * (i % 17);
            const Float d = Float{0.20} + Float{0.01} * (i % 11);
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x + 0.2, 0.2, d});
            positions.push_back(Vector3{x, 0.0, 0.0});
            positions.push_back(Vector3{x + 1.0, 0.0, 0.0});
            positions.push_back(Vector3{x, 1.0, 0.0});
            PTs.push_back(Vector4i{b, b + 1, b + 2, b + 3});
        }
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{60.0} + Float{3.0} * (i % 17);
            const Float d = Float{0.22} + Float{0.01} * (i % 11);
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x - 1.0, 0.0, 0.0});
            positions.push_back(Vector3{x + 1.0, 0.0, 0.0});
            positions.push_back(Vector3{x, -1.0, d});
            positions.push_back(Vector3{x, 1.0, d});
            EEs.push_back(Vector4i{b, b + 1, b + 2, b + 3});
        }
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{120.0} + Float{3.0} * (i % 17);
            const Float d = Float{0.24} + Float{0.01} * (i % 11);
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x, d, 0.0});
            positions.push_back(Vector3{x - 1.0, 0.0, 0.0});
            positions.push_back(Vector3{x + 1.0, 0.0, 0.0});
            PEs.push_back(Vector3i{b, b + 1, b + 2});
        }
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{180.0} + Float{3.0} * (i % 17);
            const Float d = Float{0.26} + Float{0.01} * (i % 11);
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x, 0.0, 0.0});
            positions.push_back(Vector3{x + d, 0.0, 0.0});
            PPs.push_back(Vector2i{b, b + 1});
        }
        vertex_count = static_cast<int>(positions.size());
        const std::vector<IndexT> ids(vertex_count, 0);
        const std::vector<Float>  thicknesses(vertex_count, 0.0);
        const std::vector<Float>  d_hats(vertex_count, 1.0);
        copy_prefix(d_positions, positions, vertex_count);
        copy_prefix(d_rest_positions, positions, vertex_count);
        copy_prefix(d_contact_ids, ids, vertex_count);
        copy_prefix(d_thicknesses, thicknesses, vertex_count);
        copy_prefix(d_d_hats, d_hats, vertex_count);
    }

    void compare(Counts counts)
    {
        INFO("PT=" << counts.pt << " EE=" << counts.ee << " PE=" << counts.pe
                   << " PP=" << counts.pp);
        const std::array count_array{counts.pt, counts.ee, counts.pe, counts.pp};
        constexpr std::array arities{4, 4, 3, 2};
        for(int type = 0; type < 4; ++type)
        {
            REQUIRE(count_array[type] >= 0);
            REQUIRE(count_array[type] <= MaxPerType);
            outputs[type].prepare(vertex_count, count_array[type], arities[type]);
        }
        copy_prefix(d_PTs, PTs, counts.pt);
        copy_prefix(d_EEs, EEs, counts.ee);
        copy_prefix(d_PEs, PEs, counts.pe);
        copy_prefix(d_PPs, PPs, counts.pp);

        launch_ipc_simplex_normal_contact_assembly(make_info(false));
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());

        auto check = [&]
        {
            check_outputs<4>(outputs[0], PTs, counts.pt);
            check_outputs<4>(outputs[1], EEs, counts.ee);
            check_outputs<3>(outputs[2], PEs, counts.pe);
            check_outputs<2>(outputs[3], PPs, counts.pp);
        };
        // First supply valid canary Hessian views to prove no writes occur.
        launch_ipc_simplex_normal_contact_assembly(make_info(true));
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());
        check();

        // Then use the same empty views that gradient-only assembly may receive
        // in production. Reset G so stale results cannot satisfy the comparison.
        for(auto& output : outputs)
            Outputs::reset_gradient(output.only_gradient);
        auto empty_hessian_info        = make_info(true);
        empty_hessian_info.PT_hessians = {};
        empty_hessian_info.EE_hessians = {};
        empty_hessian_info.PE_hessians = {};
        empty_hessian_info.PP_hessians = {};
        launch_ipc_simplex_normal_contact_assembly(empty_hessian_info);
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());
        check();
    }

  private:
    IPCSimplexNormalContactAssemblyLaunchInfo make_info(bool gradient_only)
    {
        return IPCSimplexNormalContactAssemblyLaunchInfo{
            .contact_tabular     = contact_tabular.view(),
            .contact_element_ids = d_contact_ids.cview(),
            .positions           = d_positions.cview(),
            .rest_positions      = d_rest_positions.cview(),
            .thicknesses         = d_thicknesses.cview(),
            .d_hats              = d_d_hats.cview(),
            .PTs                 = d_PTs.cview(),
            .EEs                 = d_EEs.cview(),
            .PEs                 = d_PEs.cview(),
            .PPs                 = d_PPs.cview(),
            .PT_gradients        = outputs[0].gradient_view(gradient_only),
            .PT_hessians         = outputs[0].hessian_view(gradient_only),
            .EE_gradients        = outputs[1].gradient_view(gradient_only),
            .EE_hessians         = outputs[1].hessian_view(gradient_only),
            .PE_gradients        = outputs[2].gradient_view(gradient_only),
            .PE_hessians         = outputs[2].hessian_view(gradient_only),
            .PP_gradients        = outputs[3].gradient_view(gradient_only),
            .PP_hessians         = outputs[3].hessian_view(gradient_only),
            .dt                  = Float{0.1},
            .gradient_only       = gradient_only};
    }

    int                          vertex_count = 0;
    DeviceBuffer2D<ContactCoeff> contact_tabular;
    DeviceBuffer<IndexT>         d_contact_ids;
    DeviceBuffer<Vector3>        d_positions, d_rest_positions;
    DeviceBuffer<Float>          d_thicknesses, d_d_hats;
    DeviceBuffer<Vector4i>       d_PTs, d_EEs;
    DeviceBuffer<Vector3i>       d_PEs;
    DeviceBuffer<Vector2i>       d_PPs;
    std::vector<Vector4i>        PTs, EEs;
    std::vector<Vector3i>        PEs;
    std::vector<Vector2i>        PPs;
    std::array<Outputs, 4>       outputs;
};
}  // namespace

TEST_CASE("IPC simplex normal-contact production gradient-only matches full assembly",
          "[cuda][contact][normal_assembly_launch][gradient_only]")
{
    CUDA_TOOL_CHECK(cudaDeviceSetLimit(cudaLimitStackSize, 32 * 1024));
    AssemblyFixture fixture;
    SECTION("zero total leaves all guards untouched")
    {
        fixture.compare({});
    }
    SECTION("individual contact types straddle CTA8 and warp boundaries")
    {
        for(int count : {1, 7, 8, 9, 31, 32, 33})
        {
            fixture.compare(Counts{.pt = count});
            fixture.compare(Counts{.ee = count});
            fixture.compare(Counts{.pe = count});
            fixture.compare(Counts{.pp = count});
        }
    }
    SECTION("mixed and empty type ranges use their distinct production layouts")
    {
        fixture.compare(Counts{.pt = 7, .ee = 8, .pe = 9, .pp = 17});
        fixture.compare(Counts{.pt = 1, .pe = 9});
        fixture.compare(Counts{.ee = 9, .pp = 1});
        fixture.compare(Counts{.pt = 9, .pp = 7});
    }
    SECTION("contiguous gradient launch crosses the maximum CUDA CTA width")
    {
        // 1028 contacts exceeds even a 1024-thread automatic CTA. Each full
        // type also ends in a one-element CTA8 tail and a padded type boundary.
        fixture.compare(Counts{257, 257, 257, 257});
    }
    SECTION("reused output capacities survive nonempty empty nonempty")
    {
        fixture.compare(Counts{.pt = 9, .ee = 17, .pe = 8, .pp = 7});
        fixture.compare({});
        fixture.compare(Counts{.pt = 1, .ee = 7, .pe = 9, .pp = 8});
    }
}
