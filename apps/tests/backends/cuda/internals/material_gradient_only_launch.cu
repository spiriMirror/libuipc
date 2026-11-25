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
#include <utils/material_gradient_hessian_launch.h>
#include <finite_element/constitutions/discrete_shell_bending_function.h>
#include <finite_element/constitutions/stress_plastic_discrete_shell_bending_function.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_function.h>
#include <affine_body/constitutions/ortho_potential_function.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;

namespace
{
constexpr Float Dt            = 0.125;
constexpr Float Sentinel      = 1234567.25;
constexpr int   IndexSentinel = -777;
// Separate/full paths can be compiled differently. Do not require bit parity.
constexpr Float AbsTolerance = 2e-11;
constexpr Float RelTolerance = 2e-10;
// CTA16/32 boundaries plus tails beyond any occupancy-selected CTA (<=1024).
constexpr std::array Counts{0, 1, 15, 16, 17, 31, 32, 33, 257, 1025};

template <typename T>
void upload(DeviceBuffer<T>& dst, const std::vector<T>& src)
{
    dst.resize(src.size());
    if(!src.empty())
        dst.view().copy_from(src.data());
}

template <typename T>
std::vector<T> download(const DeviceBuffer<T>& src)
{
    std::vector<T> result;
    src.copy_to(result);
    return result;
}

void synchronize_launch()
{
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
}

template <typename A, typename B>
void require_near(const A& actual, const B& expected)
{
    REQUIRE(actual.allFinite());
    REQUIRE(expected.allFinite());
    const Float scale =
        std::max(actual.cwiseAbs().maxCoeff(), expected.cwiseAbs().maxCoeff());
    REQUIRE((actual - expected).cwiseAbs().maxCoeff() <= AbsTolerance + RelTolerance * scale);
}

// Guard entries bracket both sparse buffers. Views have nonzero offsets, so
// this also checks publication into a subview rather than only zero-based data.
struct SparseOutput
{
    int                     vertices, gradient_count, hessian_count;
    DeviceBuffer<int>       gi, hr, hc;
    DeviceBuffer<Vector3>   g;
    DeviceBuffer<Matrix3x3> h;

    SparseOutput(int vertex_count, int ng, int nh)
        : vertices(vertex_count)
        , gradient_count(ng)
        , hessian_count(nh)
    {
        reset();
    }
    void reset()
    {
        upload(gi, std::vector<int>(gradient_count + 2, IndexSentinel));
        upload(g, std::vector<Vector3>(gradient_count + 2, Vector3::Constant(Sentinel)));
        upload(hr, std::vector<int>(hessian_count + 2, IndexSentinel));
        upload(hc, std::vector<int>(hessian_count + 2, IndexSentinel));
        upload(h, std::vector<Matrix3x3>(hessian_count + 2, Matrix3x3::Constant(Sentinel)));
    }
    DoubletVectorView<Float, 3> gradients()
    {
        return DoubletVectorView<Float, 3>{vertices, gradient_count + 2, gi.data(), g.data()}
            .subview(1, gradient_count);
    }
    TripletMatrixView<Float, 3> hessians()
    {
        return TripletMatrixView<Float, 3>{
            vertices, vertices, hessian_count + 2, hr.data(), hc.data(), h.data()}
            .subview(1, hessian_count);
    }
    void check_guards(bool untouched_h) const
    {
        const auto indices = download(gi);
        const auto values  = download(g);
        for(int slot : {0, gradient_count + 1})
        {
            REQUIRE(indices[slot] == IndexSentinel);
            REQUIRE(values[slot].isConstant(Sentinel));
        }
        const auto rows   = download(hr);
        const auto cols   = download(hc);
        const auto blocks = download(h);
        for(int slot = 0; slot < hessian_count + 2; ++slot)
            if(untouched_h || slot == 0 || slot == hessian_count + 1)
            {
                REQUIRE(rows[slot] == IndexSentinel);
                REQUIRE(cols[slot] == IndexSentinel);
                REQUIRE(blocks[slot].isConstant(Sentinel));
            }
    }
};

template <int N, typename Launch>
void compare_sparse_launches(const std::vector<Vector<int, N>>& stencils,
                             const std::vector<Vector3>&        oracle,
                             int                                vertices,
                             Launch                             launch)
{
    const int     count  = static_cast<int>(stencils.size());
    constexpr int Blocks = N * (N + 1) / 2;
    SparseOutput  full(vertices, count * N, count * Blocks);
    launch(false, full.gradients(), full.hessians());
    synchronize_launch();
    full.check_guards(false);
    const auto full_indices = download(full.gi);
    const auto full_values  = download(full.g);
    const auto rows         = download(full.hr);
    const auto cols         = download(full.hc);
    const auto hessians     = download(full.h);
    for(int i = 0; i < count; ++i)
    {
        int block = 1 + i * Blocks;
        for(int a = 0; a < N; ++a)
        {
            const int slot = 1 + i * N + a;
            REQUIRE(full_indices[slot] == stencils[i](a));
            require_near(full_values[slot], oracle[i * N + a]);
            for(int b = a; b < N; ++b, ++block)
            {
                REQUIRE(rows[block] == std::min(stencils[i](a), stencils[i](b)));
                REQUIRE(cols[block] == std::max(stencils[i](a), stencils[i](b)));
                REQUIRE(hessians[block].allFinite());
                REQUIRE((hessians[block].array() != Sentinel).all());
            }
        }
    }

    SparseOutput gradient(vertices, count * N, count * Blocks);
    for(bool empty_h : {true, false})
    {
        INFO("empty_h=" << empty_h);
        gradient.reset();
        // First use a genuinely null/empty H, then a populated canary buffer.
        launch(true,
               gradient.gradients(),
               empty_h ? TripletMatrixView<Float, 3>{} : gradient.hessians());
        synchronize_launch();
        gradient.check_guards(true);
        const auto indices = download(gradient.gi);
        const auto values  = download(gradient.g);
        for(int slot = 1; slot <= count * N; ++slot)
        {
            REQUIRE(indices[slot] == full_indices[slot]);
            require_near(values[slot], full_values[slot]);
            require_near(values[slot], oracle[slot - 1]);
        }
    }
}

void check_bending(int count, bool plastic)
{
    namespace DSB                  = sym::discrete_shell_bending;
    namespace SP                   = sym::stress_plastic_discrete_shell_bending;
    const int             vertices = 7 + 4 * count;
    std::vector<Vector4i> stencils(count);
    std::vector<Vector3> positions(vertices, Vector3::Zero()), oracle(4 * count);
    std::vector<Float> stiffness(count, 2.0), theta_bar(count, 0.15),
        heights(count, 0.8), volumes(count, 0.7), lengths(count, 1.2), yields(count);
    for(int i = 0; i < count; ++i)
    {
        const int b = 7 + 4 * i;
        volumes[i] += 0.0001 * i;
        stencils[i] = Vector4i{b + 2, b, b + 3, b + 1};
        // Same valid hinge as the existing combined-derivative helper tests.
        std::array<Vector3, 4> x = {Vector3{0, 1, 0.2},
                                    Vector3{-1, 0, 0},
                                    Vector3{1, 0, 0},
                                    Vector3{0, -1, -0.3}};
        yields[i]                = i % 3 == 1 ? 0.1 : 10.0;
        if(plastic && i % 3 == 2)
            // Existing StressPlastic guarded-degenerate fixture, not an
            // unsupported degenerate ordinary-bending input.
            x = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{2, 0, 0}, Vector3{3, 0, 0}};
        for(int a = 0; a < 4; ++a)
            positions[stencils[i](a)] = x[a];
        Vector12 g;
        if(plastic)
        {
            Float theta = 0;
            const bool valid = SP::safe_dihedral_angle(x[0], x[1], x[2], x[3], theta);
            REQUIRE(valid == (i % 3 != 2));
            if(valid)
            {
                Float delta, trial, theta_y, gamma;
                REQUIRE(SP::try_trial_state(
                    theta, theta_bar[i], stiffness[i], lengths[i], heights[i], yields[i], delta, trial, theta_y, gamma));
                REQUIRE((gamma > 0) == (i % 3 == 1));
            }
            SP::dEdx(g, x[0], x[1], x[2], x[3], lengths[i], heights[i], theta_bar[i], stiffness[i], yields[i]);
            if(!valid)
                REQUIRE(g.isZero());
        }
        else
            DSB::dEdx(g, x[0], x[1], x[2], x[3], lengths[i], heights[i], theta_bar[i], stiffness[i]);
        g *= volumes[i] * Dt * Dt;
        for(int a = 0; a < 4; ++a)
            oracle[4 * i + a] = g.segment<3>(3 * a);
    }
    DeviceBuffer<Vector4i> ds;
    DeviceBuffer<Vector3>  dx;
    DeviceBuffer<Float>    dk, dt, dh, dv, dl, dy;
    upload(ds, stencils);
    upload(dx, positions);
    upload(dk, stiffness);
    upload(dt, theta_bar);
    upload(dh, heights);
    upload(dv, volumes);
    upload(dl, lengths);
    upload(dy, yields);
    compare_sparse_launches<4>(
        stencils,
        oracle,
        vertices,
        [&](bool gradient_only, auto g, auto h)
        {
            BendingGradientHessianLaunchInfo info{ds.view(),
                                                  dk.view(),
                                                  dt.view(),
                                                  dh.view(),
                                                  dv.view(),
                                                  dl.view(),
                                                  dx.cview(),
                                                  g,
                                                  h,
                                                  Dt,
                                                  gradient_only,
                                                  dy.view()};
            if(plastic)
                launch_stress_plastic_bending_gradient_hessian(info);
            else
                launch_discrete_shell_bending_gradient_hessian(info);
        });
    // A derivative query must not perform the time-integrator's plastic update.
    REQUIRE(download(dt) == theta_bar);
    REQUIRE(download(dy) == yields);
}

void check_neo(int count)
{
    const int             vertices = 7 + 3 * count;
    std::vector<Vector3i> stencils(count);
    std::vector<Vector3> positions(vertices, Vector3::Zero()), oracle(3 * count);
    std::vector<Float> lambda(count, 2), mu(count, 3), area(count, 0.5),
        thickness(vertices, 0.02);
    std::vector<Matrix2x2> ib(count, Matrix2x2::Identity());
    for(int i = 0; i < count; ++i)
    {
        const int b = 7 + 3 * i;
        area[i] += 0.0001 * i;
        stencils[i] = Vector3i{b + 2, b, b + 1};
        // Rest, stretched, and sheared nondegenerate shell triangles.
        const Float s    = 0.01 * (i % 3);
        positions[b + 2] = Vector3{0, 0, 0};
        positions[b]     = Vector3{1 + s, s, 0};
        positions[b + 1] = Vector3{s, 1 - s, s};
        Vector9 x, g;
        for(int a = 0; a < 3; ++a)
            x.segment<3>(3 * a) = positions[stencils[i](a)];
        sym::neo_hookean_shell_2d::dEdX(g, lambda[i], mu[i], x, ib[i]);
        g *= area[i] * 2 * 0.02 * Dt * Dt;
        for(int a = 0; a < 3; ++a)
            oracle[3 * i + a] = g.segment<3>(3 * a);
    }
    DeviceBuffer<Vector3i>  ds;
    DeviceBuffer<Vector3>   dx;
    DeviceBuffer<Matrix2x2> dib;
    DeviceBuffer<Float>     dl, dm, da, dh;
    upload(ds, stencils);
    upload(dx, positions);
    upload(dib, ib);
    upload(dl, lambda);
    upload(dm, mu);
    upload(da, area);
    upload(dh, thickness);
    compare_sparse_launches<3>(
        stencils,
        oracle,
        vertices,
        [&](bool gradient_only, auto g, auto h)
        {
            launch_neo_hookean_shell_gradient_hessian(
                {dl.cview(), dm.cview(), ds.cview(), dx.cview(), dib.cview(), dh.cview(), g, h, da.cview(), Dt, gradient_only});
        });
}

void check_ortho(int count)
{
    std::vector<Vector12> qs(count), oracle(count);
    std::vector<Float>    volumes(count, 0.7), kappas(count, 2);
    for(int i = 0; i < count; ++i)
    {
        const Float s = 0.01 * (i % 3);
        volumes[i] += 0.0001 * i;
        qs[i] << 0.3, -0.2, 0.1, 1 + s, s, 0, 0, 1 - s, s, s, 0, 1;
        Vector9 g9;
        sym::abd_ortho_potential::dEdq(g9, kappas[i], qs[i]);
        oracle[i].setZero();
        oracle[i].segment<9>(3) = g9 * (volumes[i] * Dt * Dt);
    }
    DeviceBuffer<Vector12>    dq, dg;
    DeviceBuffer<Matrix12x12> dh;
    DeviceBuffer<Float>       dv, dk;
    upload(dq, qs);
    upload(dv, volumes);
    upload(dk, kappas);
    std::vector<Vector12> full;
    for(int mode : {0, 1, 2})
    {
        INFO("mode=" << mode);
        upload(dg, std::vector<Vector12>(count + 2, Vector12::Constant(Sentinel)));
        upload(dh, std::vector<Matrix12x12>(count + 2, Matrix12x12::Constant(Sentinel)));
        launch_ortho_potential_gradient_hessian(
            {dq.cview(),
             dv.cview(),
             dg.view().subview(1, count),
             mode == 1 ? BufferView<Matrix12x12>{} : dh.view().subview(1, count),
             dk.cview(),
             Dt,
             mode != 0});
        synchronize_launch();
        const auto actual   = download(dg);
        const auto hessians = download(dh);
        REQUIRE(actual.front().isConstant(Sentinel));
        REQUIRE(actual.back().isConstant(Sentinel));
        for(int slot = 0; slot < count + 2; ++slot)
        {
            if(mode != 0 || slot == 0 || slot == count + 1)
                REQUIRE(hessians[slot].isConstant(Sentinel));
            else
            {
                REQUIRE(hessians[slot].allFinite());
                REQUIRE(hessians[slot].topRows<3>().isZero());
                REQUIRE(hessians[slot].leftCols<3>().isZero());
            }
        }
        for(int i = 0; i < count; ++i)
        {
            require_near(actual[i + 1], oracle[i]);
            REQUIRE(actual[i + 1].head<3>().isZero());
            if(mode != 0)
                require_near(actual[i + 1], full[i + 1]);
        }
        if(mode == 0)
            full = actual;
    }
}
}  // namespace

TEST_CASE("ordinary bending production gradient-only launch preserves G and H canaries",
          "[cuda][material_gradient_only][production_launch]")
{
    for(int count : Counts)
    {
        INFO("count=" << count);
        check_bending(count, false);
    }
}

TEST_CASE("stress-plastic bending production gradient-only launch preserves G and history",
          "[cuda][material_gradient_only][production_launch]")
{
    for(int count : Counts)
    {
        INFO("count=" << count);
        check_bending(count, true);
    }
}

TEST_CASE("Neo-Hookean shell production gradient-only launch preserves G and H canaries",
          "[cuda][material_gradient_only][production_launch]")
{
    for(int count : Counts)
    {
        INFO("count=" << count);
        check_neo(count);
    }
}

TEST_CASE("OrthoPotential production gradient-only launch preserves G and H canaries",
          "[cuda][material_gradient_only][production_launch]")
{
    for(int count : Counts)
    {
        INFO("count=" << count);
        check_ortho(count);
    }
}
