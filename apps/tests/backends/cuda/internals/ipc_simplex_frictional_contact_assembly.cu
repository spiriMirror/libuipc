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
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <contact_system/contact_models/ipc_simplex_frictional_contact_assembly.h>
#include <cuda_tool/linear_system.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;

namespace
{
constexpr int   MaxPerType  = 16;
constexpr Float Kappa       = 3.25;
constexpr Float Dt          = 0.1;
constexpr Float EpsVelocity = 1.25;
constexpr Float Canary      = -987654.25;
constexpr int   IndexCanary = -12345;
using Counts                = std::array<int, 4>;

struct ContactCase
{
    int      kind;
    int      vertex_count;
    int      material;
    bool     mollified;
    Float    mu;
    Vector4i indices;
    Vector3  prev[4];
    Vector3  current[4];
    Vector3  rest[4];
};

struct OracleResult
{
    Vector12    gradient;
    Matrix12x12 raw_hessian;
    int         direct;
};

// Only the established local derivative helpers are used here. The oracle has
// no shared EVD, contact-range dispatch, or triplet writer: CPU Eigen below
// independently projects its dense output and checks the production writeout.
__global__ void friction_derivative_oracle(CBufferView<ContactCase> cases,
                                           BufferView<OracleResult> results)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= cases.size())
        return;
    const auto& c   = cases(i);
    auto&       out = results(i);
    out.gradient.setZero();
    out.raw_hessian.setZero();
    out.direct      = 1;
    const Float kt2 = Kappa * Dt * Dt;
    const Float eps = EpsVelocity * Dt;
    using namespace sym::codim_ipc_contact;
    if(c.kind == 0)
    {
        out.direct = PT_friction_gradient_hessian(out.gradient,
                                                  out.raw_hessian,
                                                  kt2,
                                                  1.0,
                                                  0.0,
                                                  c.mu,
                                                  eps,
                                                  c.prev[0],
                                                  c.prev[1],
                                                  c.prev[2],
                                                  c.prev[3],
                                                  c.current[0],
                                                  c.current[1],
                                                  c.current[2],
                                                  c.current[3]);
    }
    else if(c.kind == 1)
    {
        // The fixture explicitly includes parallel-edge contacts. Do not
        // evaluate their singular tangent basis in the local oracle.
        if(!c.mollified)
            out.direct = EE_friction_gradient_hessian(out.gradient,
                                                      out.raw_hessian,
                                                      kt2,
                                                      1.0,
                                                      0.0,
                                                      c.mu,
                                                      eps,
                                                      c.prev[0],
                                                      c.prev[1],
                                                      c.prev[2],
                                                      c.prev[3],
                                                      c.current[0],
                                                      c.current[1],
                                                      c.current[2],
                                                      c.current[3]);
    }
    else if(c.kind == 2)
    {
        Vector9   G;
        Matrix9x9 H;
        out.direct                            = PE_friction_gradient_hessian(G,
                                                  H,
                                                  kt2,
                                                  1.0,
                                                  0.0,
                                                  c.mu,
                                                  eps,
                                                  c.prev[0],
                                                  c.prev[1],
                                                  c.prev[2],
                                                  c.current[0],
                                                  c.current[1],
                                                  c.current[2]);
        out.gradient.head<9>()                = G;
        out.raw_hessian.topLeftCorner<9, 9>() = H;
    }
    else
    {
        Vector6   G;
        Matrix6x6 H;
        out.direct = PP_friction_gradient_hessian(
            G, H, kt2, 1.0, 0.0, c.mu, eps, c.prev[0], c.prev[1], c.current[0], c.current[1]);
        out.gradient.head<6>()                = G;
        out.raw_hessian.topLeftCorner<6, 6>() = H;
    }
}

template <typename T>
std::vector<T> download(CBufferView<T> view)
{
    std::vector<T> host(view.size());
    if(!host.empty())
        view.copy_to(host.data());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    return host;
}

bool near(Float actual, Float expected, Float scale)
{
    return std::isfinite(actual)
           && std::abs(actual - expected) <= 2.0e-9 * std::max<Float>(1.0, scale);
}

struct Output
{
    DeviceDoubletVector<Float, 3> G;
    DeviceTripletMatrix<Float, 3> H;
    int                           g_count = 0;
    int                           h_count = 0;

    void reset(int vertices, int count, int stencil_size)
    {
        g_count = count * stencil_size;
        h_count = count * stencil_size * (stencil_size + 1) / 2;
        // A trailing entry is outside the passed view, including when empty.
        G.resize(vertices, g_count + 1);
        H.resize(vertices, vertices, h_count + 1);
        BufferLaunch().fill(G.indices(), IndexCanary);
        BufferLaunch().fill(G.values(), Vector3::Constant(Canary).eval());
        BufferLaunch().fill(H.row_indices(), IndexCanary);
        BufferLaunch().fill(H.col_indices(), IndexCanary);
        BufferLaunch().fill(H.values(), Matrix3x3::Constant(Canary).eval());
    }

    auto gradients() { return G.view().subview(0, g_count); }
    auto hessians() { return H.view().subview(0, h_count); }

    void check_canaries(bool hessian_untouched)
    {
        const auto gi = download<IndexT>(G.indices());
        const auto gv = download<Vector3>(G.values());
        const auto hr = download<IndexT>(H.row_indices());
        const auto hc = download<IndexT>(H.col_indices());
        const auto hv = download<Matrix3x3>(H.values());
        CHECK(gi.back() == IndexCanary);
        CHECK((gv.back().array() == Canary).all());
        for(int i = hessian_untouched ? 0 : h_count; i <= h_count; ++i)
        {
            CHECK(hr[i] == IndexCanary);
            CHECK(hc[i] == IndexCanary);
            CHECK((hv[i].array() == Canary).all());
        }
    }
};

class FrictionAssemblyFixture
{
  public:
    FrictionAssemblyFixture()
        : table(Extent2D{3, 3})
    {
        const Float                 mus[] = {0.65, 0.0, -0.65};
        std::array<ContactCoeff, 9> coeffs;
        for(int row = 0; row < 3; ++row)
            for(int col = 0; col < 3; ++col)
                coeffs[row * 3 + col] = ContactCoeff{Kappa, mus[row]};
        table.view().copy_from(coeffs.data());

        std::vector<Vector3> positions;
        std::vector<Vector3> previous;
        std::vector<Vector3> rest;
        std::vector<IndexT>  materials;
        for(int kind = 0; kind < 4; ++kind)
        {
            for(int i = 0; i < MaxPerType; ++i)
            {
                ContactCase c{};
                c.kind         = kind;
                c.vertex_count = kind < 2 ? 4 : (kind == 2 ? 3 : 2);
                c.material     = i % 3;
                c.mu           = mus[c.material];
                c.mollified    = kind == 1 && i >= 12;
                if(kind == 0)
                {
                    c.prev[0] = Vector3{0.25, 0.25, 0.25};
                    c.prev[1] = Vector3{0.0, 0.0, 0.0};
                    c.prev[2] = Vector3{1.0, 0.0, 0.0};
                    c.prev[3] = Vector3{0.0, 1.0, 0.0};
                }
                else if(kind == 1)
                {
                    c.prev[0] = Vector3{-1.0, 0.0, 0.0};
                    c.prev[1] = Vector3{1.0, 0.0, 0.0};
                    c.prev[2] = c.mollified ? Vector3{-1.0, 0.0, 0.25} :
                                              Vector3{0.0, -1.0, 0.25};
                    c.prev[3] = c.mollified ? Vector3{1.0, 0.0, 0.25} :
                                              Vector3{0.0, 1.0, 0.25};
                }
                else if(kind == 2)
                {
                    c.prev[0] = Vector3{0.0, 0.25, 0.0};
                    c.prev[1] = Vector3{-1.0, 0.0, 0.0};
                    c.prev[2] = Vector3{1.0, 0.0, 0.0};
                }
                else
                {
                    c.prev[0] = Vector3{0.0, 0.0, 0.0};
                    c.prev[1] = Vector3{0.25, 0.0, 0.0};
                }
                for(int j = 0; j < c.vertex_count; ++j)
                    c.current[j] = c.rest[j] = c.prev[j];
                const Float radii[] = {0.0, 0.5, 1.0, 2.0};
                const Float slip    = radii[(i / 3) % 4] * EpsVelocity * Dt;
                c.current[0](kind == 3 ? 1 : 0) += slip;
                if(kind == 1)
                    c.current[1](0) += slip;

                const int base = static_cast<int>(positions.size());
                positions.resize(base + c.vertex_count);
                previous.resize(base + c.vertex_count);
                rest.resize(base + c.vertex_count);
                materials.resize(base + c.vertex_count, c.material);
                // Deliberately non-monotone global IDs exercise upper_LR and
                // block transposition, without changing the local geometry.
                const int permutations[3][4] = {{1, 0, 0, 0}, {1, 2, 0, 0}, {2, 0, 3, 1}};
                for(int j = 0; j < c.vertex_count; ++j)
                {
                    const int index = base + permutations[c.vertex_count - 2][j];
                    c.indices[j]     = index;
                    positions[index] = c.current[j];
                    previous[index]  = c.prev[j];
                    rest[index]      = c.rest[j];
                }
                cases.push_back(c);
                if(kind == 0)
                    pts.push_back(c.indices);
                else if(kind == 1)
                    ees.push_back(c.indices);
                else if(kind == 2)
                    pes.push_back(c.indices.head<3>());
                else
                    pps.push_back(c.indices.head<2>());
            }
        }
        vertex_count = static_cast<int>(positions.size());
        d_positions.copy_from(positions.data(), positions.size());
        d_previous.copy_from(previous.data(), previous.size());
        d_rest.copy_from(rest.data(), rest.size());
        d_materials.copy_from(materials.data(), materials.size());
        d_thickness.resize(vertex_count, 0.0);
        d_dhat.resize(vertex_count, 1.0);
        d_cases.copy_from(cases.data(), cases.size());
        d_oracle.resize(cases.size());
        friction_derivative_oracle<<<(cases.size() + 31) / 32, 32>>>(d_cases.cview(),
                                                                     d_oracle.view());
        CUDA_TOOL_CHECK(cudaGetLastError());
        oracle = download<OracleResult>(d_oracle.cview());
    }

    void compare(Counts counts)
    {
        for(int count : counts)
        {
            REQUIRE(count >= 0);
            REQUIRE(count <= MaxPerType);
        }
        set_prefix(d_pts, pts, counts[0]);
        set_prefix(d_ees, ees, counts[1]);
        set_prefix(d_pes, pes, counts[2]);
        set_prefix(d_pps, pps, counts[3]);
        d_lagged.resize(counts[0] + counts[1] + counts[2] + counts[3]);
        for(int type = 0; type < 4; ++type)
        {
            const int stencil = type < 2 ? 4 : (type == 2 ? 3 : 2);
            full[type].reset(vertex_count, counts[type], stencil);
            gradient[type].reset(vertex_count, counts[type], stencil);
        }
        launch_ipc_simplex_frictional_contact_assembly(make_info(full, false));
        CUDA_TOOL_CHECK(cudaGetLastError());
        launch_ipc_simplex_frictional_contact_assembly(make_info(gradient, true));
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());

        for(int type = 0; type < 4; ++type)
        {
            INFO("contact type=" << type << ", count=" << counts[type]);
            full[type].check_canaries(false);
            gradient[type].check_canaries(true);
            compare_type(type, counts[type]);
        }

        // The actual production gradient-only contract also permits null H
        // views, not just valid allocations that happen to stay untouched.
        // Re-poison G as well, so a skipped launch cannot pass using the
        // preceding gradient-only result.
        for(int type = 0; type < 4; ++type)
        {
            const int stencil = type < 2 ? 4 : (type == 2 ? 3 : 2);
            gradient[type].reset(vertex_count, counts[type], stencil);
        }
        auto null_h        = make_info(gradient, true);
        null_h.PT_hessians = {};
        null_h.EE_hessians = {};
        null_h.PE_hessians = {};
        null_h.PP_hessians = {};
        launch_ipc_simplex_frictional_contact_assembly(null_h);
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());
        for(int type = 0; type < 4; ++type)
        {
            gradient[type].check_canaries(true);
            compare_type(type, counts[type]);
        }
    }

  private:
    template <typename T>
    static void set_prefix(DeviceBuffer<T>& dst, const std::vector<T>& src, int n)
    {
        dst.resize(n);
        if(n)
            dst.view().copy_from(src.data());
    }

    IPCSimplexFrictionalContactAssemblyLaunchInfo make_info(std::array<Output, 4>& output,
                                                            bool gradient_only)
    {
        return {.contact_tabular     = table.cview(),
                .contact_element_ids = d_materials.cview(),
                .positions           = d_positions.cview(),
                .prev_positions      = d_previous.cview(),
                .rest_positions      = d_rest.cview(),
                .thicknesses         = d_thickness.cview(),
                .d_hats              = d_dhat.cview(),
                .PTs                 = d_pts.cview(),
                .EEs                 = d_ees.cview(),
                .PEs                 = d_pes.cview(),
                .PPs                 = d_pps.cview(),
                .PT_gradients        = output[0].gradients(),
                .PT_hessians         = output[0].hessians(),
                .EE_gradients        = output[1].gradients(),
                .EE_hessians         = output[1].hessians(),
                .PE_gradients        = output[2].gradients(),
                .PE_hessians         = output[2].hessians(),
                .PP_gradients        = output[3].gradients(),
                .PP_hessians         = output[3].hessians(),
                .lagged              = d_lagged.view(),
                .eps_velocity        = EpsVelocity,
                .dt                  = Dt,
                .gradient_only       = gradient_only};
    }

    void compare_type(int type, int count)
    {
        const auto gi       = download<IndexT>(full[type].G.indices());
        const auto gg       = download<Vector3>(full[type].G.values());
        const auto only_i   = download<IndexT>(gradient[type].G.indices());
        const auto only_g   = download<Vector3>(gradient[type].G.values());
        const auto hr       = download<IndexT>(full[type].H.row_indices());
        const auto hc       = download<IndexT>(full[type].H.col_indices());
        const auto hv       = download<Matrix3x3>(full[type].H.values());
        int        g_offset = 0;
        int        h_offset = 0;
        for(int i = 0; i < count; ++i)
        {
            const auto& c         = cases[type * MaxPerType + i];
            const auto& reference = oracle[type * MaxPerType + i];
            INFO("case=" << i << ", mu=" << c.mu << ", mollified=" << c.mollified);
            REQUIRE(reference.gradient.allFinite());
            REQUIRE(reference.raw_hessian.allFinite());
            REQUIRE(reference.direct == (c.mu >= 0.0 || c.mollified ? 1 : 0));
            const int       n   = 3 * c.vertex_count;
            Eigen::MatrixXd raw = reference.raw_hessian.topLeftCorner(n, n);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(raw);
            REQUIRE(solver.info() == Eigen::Success);
            Eigen::MatrixXd projected =
                solver.eigenvectors() * solver.eigenvalues().cwiseMax(0.0).asDiagonal()
                * solver.eigenvectors().transpose();
            const Float scale = std::max<Float>(1.0, raw.cwiseAbs().maxCoeff());
            if(c.mu < 0.0 && !c.mollified)
            {
                // Non-physical defensive input: prove that this actually
                // needs projection, so accidentally taking the direct path
                // cannot make a vacuous fallback test pass.
                REQUIRE(solver.eigenvalues().minCoeff() < -1.0e-5);
                REQUIRE(projected.cwiseAbs().maxCoeff() < 2.0e-9 * scale);
            }
            Eigen::MatrixXd assembled = Eigen::MatrixXd::Zero(n, n);
            for(int j = 0; j < c.vertex_count; ++j, ++g_offset)
            {
                CHECK(gi[g_offset] == c.indices[j]);
                CHECK(only_i[g_offset] == c.indices[j]);
                for(int axis = 0; axis < 3; ++axis)
                {
                    const Float expected = reference.gradient[3 * j + axis];
                    const Float g_scale = std::max<Float>(1.0, std::abs(expected));
                    CHECK(near(gg[g_offset][axis], expected, g_scale));
                    CHECK(near(only_g[g_offset][axis], expected, g_scale));
                    CHECK(near(only_g[g_offset][axis], gg[g_offset][axis], g_scale));
                }
            }
            for(int j = 0; j < c.vertex_count; ++j)
                for(int k = j; k < c.vertex_count; ++k, ++h_offset)
                {
                    int row = j;
                    int col = k;
                    if(c.indices[row] > c.indices[col])
                        std::swap(row, col);
                    CHECK(hr[h_offset] == c.indices[row]);
                    CHECK(hc[h_offset] == c.indices[col]);
                    REQUIRE(hv[h_offset].allFinite());
                    for(int r = 0; r < 3; ++r)
                        for(int s = 0; s < 3; ++s)
                            CHECK(near(hv[h_offset](r, s),
                                       projected(3 * row + r, 3 * col + s),
                                       scale));
                    assembled.block<3, 3>(3 * row, 3 * col) = hv[h_offset];
                    if(row != col)
                        assembled.block<3, 3>(3 * col, 3 * row) =
                            hv[h_offset].transpose();
                }
            CHECK((assembled - assembled.transpose()).cwiseAbs().maxCoeff() <= 2.0e-9 * scale);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> psd(assembled);
            REQUIRE(psd.info() == Eigen::Success);
            CHECK(psd.eigenvalues().minCoeff() >= -2.0e-9 * scale);
        }
        CHECK(g_offset == full[type].g_count);
        CHECK(h_offset == full[type].h_count);
    }

    int                                    vertex_count = 0;
    std::vector<ContactCase>               cases;
    std::vector<OracleResult>              oracle;
    std::vector<Vector4i>                  pts, ees;
    std::vector<Vector3i>                  pes;
    std::vector<Vector2i>                  pps;
    DeviceBuffer2D<ContactCoeff>           table;
    DeviceBuffer<IndexT>                   d_materials;
    DeviceBuffer<Vector3>                  d_positions, d_previous, d_rest;
    DeviceBuffer<Float>                    d_thickness, d_dhat;
    DeviceBuffer<Vector4i>                 d_pts, d_ees;
    DeviceBuffer<Vector3i>                 d_pes;
    DeviceBuffer<Vector2i>                 d_pps;
    DeviceBuffer<IPCSimplexFrictionLagged> d_lagged;
    DeviceBuffer<ContactCase>              d_cases;
    DeviceBuffer<OracleResult>             d_oracle;
    std::array<Output, 4>                  full, gradient;
};
}  // namespace

TEST_CASE("IPC friction production assembly matches dense PSD and gradient oracles",
          "[cuda][contact][friction][production_assembly]")
{
    FrictionAssemblyFixture fixture;
    SECTION("all empty")
    {
        fixture.compare({0, 0, 0, 0});
    }
    SECTION("individual types straddle CTA12 tails")
    {
        for(int type = 0; type < 4; ++type)
            for(int count : {1, 11, 12, 13, 16})
            {
                Counts counts{};
                counts[type] = count;
                fixture.compare(counts);
            }
    }
    SECTION("mixed types and finite negative-scale defensive fallbacks")
    {
        fixture.compare({13, 16, 11, 12});
        fixture.compare({1, 1, 1, 1});
    }
    SECTION("empty interior types and reused capacities")
    {
        fixture.compare({16, 16, 16, 16});
        fixture.compare({13, 0, 13, 0});
        fixture.compare({0, 16, 0, 13});
        fixture.compare({0, 0, 0, 0});
        fixture.compare({3, 4, 5, 6});
    }
}
