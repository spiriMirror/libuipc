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
#include <linear_system/fused_pcg_kernels.h>

#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
struct FusedPcgState
{
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> p;
    cuda_tool::DeviceDenseVector<Float> r;
    cuda_tool::DeviceDenseVector<Float> Ap;
    cuda_tool::DeviceDenseVector<Float> z;
    cuda_tool::DeviceVar<Float>         rz{Float{3}};
    cuda_tool::DeviceVar<Float>         pAp{Float{2}};
    cuda_tool::DeviceVar<Float>         rz_new{Float{0}};
    cuda_tool::DeviceVar<IndexT>        converged{IndexT{0}};

    explicit FusedPcgState(SizeT n)
    {
        std::vector<Float> host(n);
        auto fill = [&](cuda_tool::DeviceDenseVector<Float>& v, Float a, int period, Float b)
        {
            for(SizeT i = 0; i < n; ++i)
                host[i] = a * Float(i % period + 1) + b;
            v.resize(n);
            v.buffer_view().copy_from(host.data());
        };
        fill(x, 0.01, 29, 0.0);
        fill(p, 0.02, 7, -0.05);
        fill(r, 0.003, 31, 0.4);
        fill(Ap, 0.005, 5, 0.0);
        fill(z, 0.0, 1, -1.0);
    }

    std::vector<Float> host(const cuda_tool::DeviceDenseVector<Float>& v) const
    {
        std::vector<Float> out(v.size());
        v.buffer_view().copy_to(out.data());
        return out;
    }
};

template <typename Block>
FusedPcgBlockDiagSegment<Block> segment(const cuda_tool::DeviceBuffer<Block>& inv,
                                        FusedPcgState& s,
                                        SizeT          offset,
                                        SizeT          count)
{
    return {inv.cview(),
            s.x.view().subview(offset, count),
            s.p.cview().subview(offset, count),
            s.r.view().subview(offset, count),
            s.Ap.cview().subview(offset, count),
            s.z.view().subview(offset, count)};
}
}  // namespace

TEST_CASE("fused PCG block-diagonal launch matches per-subsystem launches",
          "[build_solve_focused][fused_pcg][block_diag_fused]")
{
    // 5 bodies followed by 70 vertices: the 3x3 part ends in a partial block.
    constexpr int Bodies   = 5;
    constexpr int Vertices = 70;
    constexpr int Dofs12   = Bodies * 12;
    constexpr int Dofs3    = Vertices * 3;
    constexpr int Dofs     = Dofs12 + Dofs3;

    std::vector<Matrix12x12> inv12(Bodies);
    for(int b = 0; b < Bodies; ++b)
        for(int i = 0; i < 12; ++i)
            for(int j = 0; j < 12; ++j)
                inv12[b](i, j) = i == j ? 0.5 + 0.01 * b + 0.005 * i :
                                          0.001 * ((i + 2 * j + b) % 5) - 0.002;
    std::vector<Matrix3x3> inv3(Vertices);
    for(int v = 0; v < Vertices; ++v)
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                inv3[v](i, j) = i == j ? 0.8 + 0.003 * v : 0.01 * ((v + i + j) % 3);

    cuda_tool::DeviceBuffer<Matrix12x12> d_inv12(Bodies);
    cuda_tool::DeviceBuffer<Matrix3x3>   d_inv3(Vertices);
    d_inv12.view().copy_from(inv12.data());
    d_inv3.view().copy_from(inv3.data());

    FusedPcgState separate(Dofs);
    {
        auto s12 = segment(d_inv12, separate, 0, Dofs12);
        auto s3  = segment(d_inv3, separate, Dofs12, Dofs3);
        launch_fused_pcg_abd_update_apply_dot(d_inv12.cview(),
                                              s12.x,
                                              s12.p,
                                              s12.r,
                                              s12.Ap,
                                              s12.z,
                                              separate.rz.view(),
                                              separate.pAp.view(),
                                              separate.rz_new.view(),
                                              separate.converged.view(),
                                              nullptr);
        launch_fused_pcg_fem_update_apply_dot(d_inv3.cview(),
                                              s3.x,
                                              s3.p,
                                              s3.r,
                                              s3.Ap,
                                              s3.z,
                                              separate.rz.view(),
                                              separate.pAp.view(),
                                              separate.rz_new.view(),
                                              separate.converged.view(),
                                              nullptr);
    }

    FusedPcgState combined(Dofs);
    launch_fused_pcg_block_diag_update_apply_dot(segment(d_inv12, combined, 0, Dofs12),
                                                 segment(d_inv3, combined, Dofs12, Dofs3),
                                                 combined.rz.view(),
                                                 combined.pAp.view(),
                                                 combined.rz_new.view(),
                                                 combined.converged.view(),
                                                 nullptr);

    // Every block runs the same per-block code, so vectors are bit-identical;
    // only the order of the two-or-more partial dot atomics may differ.
    REQUIRE(combined.host(combined.x) == separate.host(separate.x));
    REQUIRE(combined.host(combined.r) == separate.host(separate.r));
    REQUIRE(combined.host(combined.z) == separate.host(separate.z));
    const Float expected_rz_new = static_cast<Float>(separate.rz_new);
    REQUIRE(expected_rz_new != Float{0});
    REQUIRE(static_cast<Float>(combined.rz_new)
            == Catch::Approx(expected_rz_new).epsilon(1e-13));

    SECTION("converged status is a full no-op")
    {
        FusedPcgState done(Dofs);
        done.converged = IndexT{1};
        const auto x0  = done.host(done.x);
        const auto r0  = done.host(done.r);
        const auto z0  = done.host(done.z);
        launch_fused_pcg_block_diag_update_apply_dot(segment(d_inv12, done, 0, Dofs12),
                                                     segment(d_inv3, done, Dofs12, Dofs3),
                                                     done.rz.view(),
                                                     done.pAp.view(),
                                                     done.rz_new.view(),
                                                     done.converged.view(),
                                                     nullptr);
        REQUIRE(done.host(done.x) == x0);
        REQUIRE(done.host(done.r) == r0);
        REQUIRE(done.host(done.z) == z0);
        REQUIRE(static_cast<Float>(done.rz_new) == Float{0});
    }

    SECTION("an empty 12x12 segment leaves only the 3x3 blocks")
    {
        FusedPcgState                        only3(Dofs3);
        FusedPcgState                        reference(Dofs3);
        cuda_tool::DeviceBuffer<Matrix12x12> no_bodies;
        auto ref3 = segment(d_inv3, reference, 0, Dofs3);
        launch_fused_pcg_fem_update_apply_dot(d_inv3.cview(),
                                              ref3.x,
                                              ref3.p,
                                              ref3.r,
                                              ref3.Ap,
                                              ref3.z,
                                              reference.rz.view(),
                                              reference.pAp.view(),
                                              reference.rz_new.view(),
                                              reference.converged.view(),
                                              nullptr);
        launch_fused_pcg_block_diag_update_apply_dot(segment(no_bodies, only3, 0, 0),
                                                     segment(d_inv3, only3, 0, Dofs3),
                                                     only3.rz.view(),
                                                     only3.pAp.view(),
                                                     only3.rz_new.view(),
                                                     only3.converged.view(),
                                                     nullptr);
        REQUIRE(only3.host(only3.z) == reference.host(reference.z));
        REQUIRE(static_cast<Float>(only3.rz_new)
                == Catch::Approx(static_cast<Float>(reference.rz_new)).epsilon(1e-13));
    }
}
