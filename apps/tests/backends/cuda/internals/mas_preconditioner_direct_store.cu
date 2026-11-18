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
#include <finite_element/mas_preconditioner_engine.h>
#include <cub/block/block_reduce.cuh>
#include <cub/warp/warp_reduce.cuh>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
constexpr int VertexCount = MASPreconditionerEngine::BANKSIZE;
constexpr int ScalarCount = VertexCount * 3;

std::vector<Float> expected_diagonal_apply(const std::vector<Float>& r)
{
    std::vector<Float> expected(ScalarCount);
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        for(int component = 0; component < 3; ++component)
        {
            const Float diagonal = Float{2 + component} + Float{0.125} * vertex;
            expected[3 * vertex + component] = r[3 * vertex + component] / diagonal;
        }
    }
    return expected;
}

void require_near(const std::vector<Float>& actual,
                  const std::vector<Float>& expected,
                  Float                     tolerance = 3e-5)
{
    REQUIRE(actual.size() == expected.size());
    for(SizeT i = 0; i < actual.size(); ++i)
        REQUIRE(actual[i] == Catch::Approx(expected[i]).margin(tolerance));
}
}  // namespace

namespace
{
// The former separate diagonal pass of FEMMASPreconditioner::do_apply, kept
// as the bitwise reference for the fallback that apply() now serves in its
// prolongation pass.
__global__ void reference_apply_diag_inv_for_unpartitioned_kernel(
    cuda_tool::CDenseVectorView<Float> r_view,
    cuda_tool::DenseVectorView<Float>  z_view,
    cuda_tool::CBufferView<Matrix3x3>  diag,
    cuda_tool::CBufferView<int>        unpart,
    const IndexT*                      converged,
    int                                n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    if(*converged != 0)
        return;
    if(unpart(i) == 1)
    {
        z_view.segment<3>(i * 3).as_eigen() =
            diag(i) * r_view.segment<3>(i * 3).as_eigen();
    }
}
}  // namespace

TEST_CASE("MAS local solve direct stores replace cleared atomic accumulation",
          "[build_solve_focused][mas][direct_store]")
{
    std::vector<unsigned int> neighbor_starts(VertexCount, 0);
    std::vector<unsigned int> neighbor_counts(VertexCount, 0);
    std::vector<unsigned int> neighbor_list;
    std::vector<int>          part_to_real(VertexCount);
    std::vector<int>          real_to_part(VertexCount);
    std::iota(part_to_real.begin(), part_to_real.end(), 0);
    std::iota(real_to_part.begin(), real_to_part.end(), 0);

    MASPreconditionerEngine engine;
    engine.init_neighbor(VertexCount,
                         0,
                         VertexCount,
                         span<const unsigned int>{neighbor_list},
                         span<const unsigned int>{neighbor_starts},
                         span<const unsigned int>{neighbor_counts},
                         span<const int>{part_to_real},
                         span<const int>{real_to_part});
    engine.init_matrix();
    engine.set_active_level_num(1);

    std::array<Matrix3x3, VertexCount> values;
    std::array<int, VertexCount>       rows;
    std::array<int, VertexCount>       cols;
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        values[vertex].setZero();
        for(int component = 0; component < 3; ++component)
            values[vertex](component, component) =
                Float{2 + component} + Float{0.125} * vertex;
        rows[vertex] = vertex;
        cols[vertex] = vertex;
    }

    cuda_tool::DeviceBuffer<Matrix3x3> d_values(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_rows(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_cols(VertexCount);
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    d_cols.view().copy_from(cols.data());
    engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);

    cuda_tool::DeviceDenseVector<Float> d_r;
    cuda_tool::DeviceDenseVector<Float> d_z;
    cuda_tool::DeviceVar<IndexT>        d_converged{IndexT{0}};
    d_r.resize(ScalarCount);
    d_z.resize(ScalarCount);

    std::vector<Float> actual(ScalarCount);
    for(int pass = 0; pass < 2; ++pass)
    {
        std::vector<Float> residual(ScalarCount);
        for(int i = 0; i < ScalarCount; ++i)
            residual[i] = Float{0.25 * (pass + 1)} + Float{0.03125} * (i + 1);

        d_r.buffer_view().copy_from(residual.data());
        d_z.buffer_view().fill(Float{-100 - pass});
        engine.apply(d_r.cview(), d_z.view(), d_converged.view(), nullptr);
        d_z.copy_to(actual);
        require_near(actual, expected_diagonal_apply(residual));
    }

    SECTION("converged apply leaves z untouched")
    {
        d_z.buffer_view().fill(Float{17});
        d_converged = IndexT{1};
        engine.apply(d_r.cview(), d_z.view(), d_converged.view(), nullptr);
        d_z.copy_to(actual);
        require_near(actual, std::vector<Float>(ScalarCount, Float{17}), Float{0});
    }
}

TEST_CASE("MAS fused PCG update apply dot matches the reference arithmetic",
          "[build_solve_focused][mas][fused_pcg]")
{
    std::vector<unsigned int> neighbor_starts(VertexCount, 0);
    std::vector<unsigned int> neighbor_counts(VertexCount, 0);
    std::vector<unsigned int> neighbor_list;
    std::vector<int>          part_to_real(VertexCount);
    std::vector<int>          real_to_part(VertexCount);
    std::iota(part_to_real.begin(), part_to_real.end(), 0);
    std::iota(real_to_part.begin(), real_to_part.end(), 0);

    MASPreconditionerEngine engine;
    engine.init_neighbor(VertexCount,
                         0,
                         VertexCount,
                         span<const unsigned int>{neighbor_list},
                         span<const unsigned int>{neighbor_starts},
                         span<const unsigned int>{neighbor_counts},
                         span<const int>{part_to_real},
                         span<const int>{real_to_part});
    engine.init_matrix();
    engine.set_active_level_num(1);

    std::array<Matrix3x3, VertexCount> values;
    std::array<int, VertexCount>       rows;
    std::array<int, VertexCount>       cols;
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        values[vertex].setZero();
        for(int component = 0; component < 3; ++component)
            values[vertex](component, component) =
                Float{2 + component} + Float{0.125} * vertex;
        rows[vertex] = vertex;
        cols[vertex] = vertex;
    }

    cuda_tool::DeviceBuffer<Matrix3x3> d_values(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_rows(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_cols(VertexCount);
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    d_cols.view().copy_from(cols.data());
    engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);

    cuda_tool::DeviceDenseVector<Float> d_x;
    cuda_tool::DeviceDenseVector<Float> d_p;
    cuda_tool::DeviceDenseVector<Float> d_r;
    cuda_tool::DeviceDenseVector<Float> d_Ap;
    cuda_tool::DeviceDenseVector<Float> d_z;
    d_x.resize(ScalarCount);
    d_p.resize(ScalarCount);
    d_r.resize(ScalarCount);
    d_Ap.resize(ScalarCount);
    d_z.resize(ScalarCount);

    constexpr Float              Rz           = 3.0;
    constexpr Float              PAp          = 2.0;
    constexpr Float              Alpha        = Rz / PAp;
    constexpr Float              InitialRzNew = 0.75;
    cuda_tool::DeviceVar<Float>  d_rz{Rz};
    cuda_tool::DeviceVar<Float>  d_pAp{PAp};
    cuda_tool::DeviceVar<Float>  d_rz_new{InitialRzNew};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    std::vector<Float> actual_x(ScalarCount);
    std::vector<Float> actual_r(ScalarCount);
    std::vector<Float> actual_z(ScalarCount);
    for(int pass = 0; pass < 2; ++pass)
    {
        std::vector<Float> x(ScalarCount);
        std::vector<Float> p(ScalarCount);
        std::vector<Float> r(ScalarCount);
        std::vector<Float> Ap(ScalarCount);
        std::vector<Float> expected_x(ScalarCount);
        std::vector<Float> expected_r(ScalarCount);
        for(int i = 0; i < ScalarCount; ++i)
        {
            x[i]          = Float{0.01 * (i + 1) + 0.1 * pass};
            p[i]          = Float{0.02 * (i % 7 + 1)};
            r[i]          = Float{0.4 + 0.003 * i + 0.2 * pass};
            Ap[i]         = Float{0.005 * (i % 5 + 1)};
            expected_x[i] = x[i] + Alpha * p[i];
            expected_r[i] = r[i] - Alpha * Ap[i];
        }
        const auto expected_z      = expected_diagonal_apply(expected_r);
        Float      expected_rz_new = InitialRzNew;
        for(int i = 0; i < ScalarCount; ++i)
            expected_rz_new += expected_r[i] * expected_z[i];

        d_x.buffer_view().copy_from(x.data());
        d_p.buffer_view().copy_from(p.data());
        d_r.buffer_view().copy_from(r.data());
        d_Ap.buffer_view().copy_from(Ap.data());
        d_z.buffer_view().fill(Float{-100 - pass});
        d_rz_new = InitialRzNew;

        engine.fused_pcg_update_apply_dot(d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          nullptr);

        d_x.buffer_view().copy_to(actual_x.data());
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        require_near(actual_x, expected_x, 1e-12);
        require_near(actual_r, expected_r, 1e-12);
        require_near(actual_z, expected_z);
        REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).margin(1e-4));
    }

    SECTION("converged status is a full no-op")
    {
        d_x.buffer_view().fill(Float{7});
        d_r.buffer_view().fill(Float{11});
        d_z.buffer_view().fill(Float{13});
        d_rz_new    = Float{17};
        d_converged = IndexT{1};

        engine.fused_pcg_update_apply_dot(d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          nullptr);

        d_x.buffer_view().copy_to(actual_x.data());
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        require_near(actual_x, std::vector<Float>(ScalarCount, Float{7}), Float{0});
        require_near(actual_r, std::vector<Float>(ScalarCount, Float{11}), Float{0});
        require_near(actual_z, std::vector<Float>(ScalarCount, Float{13}), Float{0});
        REQUIRE(static_cast<Float>(d_rz_new) == Float{17});
    }
}

TEST_CASE("MAS fused PCG step serves unpartitioned vertices with the diagonal fallback",
          "[build_solve_focused][mas][fused_pcg]")
{
    // One full cluster of partitioned vertices plus unpartitioned vertices
    // interleaved in real order, as Empty-constitution FEM vertices appear
    // next to partitioned meshes.
    constexpr int            Unpartitioned = 5;
    constexpr int            Vertices      = VertexCount + Unpartitioned;
    constexpr int            Scalars       = Vertices * 3;
    const std::array<int, 5> unpartitioned = {3, 7, 11, 18, 20};

    std::vector<int> part_to_real;
    std::vector<int> real_to_part(Vertices, -1);
    for(int vertex = 0; vertex < Vertices; ++vertex)
    {
        if(std::find(unpartitioned.begin(), unpartitioned.end(), vertex)
           != unpartitioned.end())
            continue;
        real_to_part[vertex] = static_cast<int>(part_to_real.size());
        part_to_real.push_back(vertex);
    }
    REQUIRE(part_to_real.size() == VertexCount);

    std::vector<unsigned int> neighbor_starts(Vertices, 0);
    std::vector<unsigned int> neighbor_counts(Vertices, 0);
    std::vector<unsigned int> neighbor_list;

    MASPreconditionerEngine engine;
    engine.init_neighbor(Vertices,
                         0,
                         VertexCount,
                         span<const unsigned int>{neighbor_list},
                         span<const unsigned int>{neighbor_starts},
                         span<const unsigned int>{neighbor_counts},
                         span<const int>{part_to_real},
                         span<const int>{real_to_part});
    engine.init_matrix();
    engine.set_active_level_num(1);

    // Diagonal Hessian blocks for every vertex; the MAS scatter ignores the
    // unpartitioned ones. Partitioned vertex v (cluster lane l) gets the
    // diagonal of expected_diagonal_apply's lane l.
    std::vector<Matrix3x3> values(Vertices, Matrix3x3::Zero());
    std::vector<int>       rows(Vertices);
    for(int vertex = 0; vertex < Vertices; ++vertex)
    {
        const int lane = real_to_part[vertex] >= 0 ? real_to_part[vertex] : 0;
        for(int component = 0; component < 3; ++component)
            values[vertex](component, component) =
                Float{2 + component} + Float{0.125} * lane;
        rows[vertex] = vertex;
    }
    cuda_tool::DeviceBuffer<Matrix3x3> d_values(Vertices);
    cuda_tool::DeviceBuffer<int>       d_rows(Vertices);
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_rows.cview(), 0);

    // Full (non-diagonal) SPD inverse blocks for the diagonal fallback; the
    // partitioned entries must not be used.
    std::vector<Matrix3x3> diag_inv(Vertices, Matrix3x3::Constant(Float{1e30}));
    for(int vertex : unpartitioned)
    {
        Matrix3x3 m;
        m << 2.0, 0.25, 0.125, 0.25, 3.0, 0.5, 0.125, 0.5, 4.0 + 0.1 * vertex;
        diag_inv[vertex] = m;
    }
    cuda_tool::DeviceBuffer<Matrix3x3> d_diag_inv(Vertices);
    d_diag_inv.view().copy_from(diag_inv.data());

    constexpr Float Rz           = 3.0;
    constexpr Float PAp          = 2.0;
    constexpr Float Alpha        = Rz / PAp;
    constexpr Float InitialRzNew = 0.75;

    std::vector<Float> x(Scalars), p(Scalars), r(Scalars), Ap(Scalars);
    std::vector<Float> expected_x(Scalars), expected_r(Scalars), expected_z(Scalars);
    for(int i = 0; i < Scalars; ++i)
    {
        x[i]          = Float{0.01 * (i + 1)};
        p[i]          = Float{0.02 * (i % 7 + 1)};
        r[i]          = Float{0.4 + 0.003 * i};
        Ap[i]         = Float{0.005 * (i % 5 + 1)};
        expected_x[i] = x[i] + Alpha * p[i];
        expected_r[i] = r[i] - Alpha * Ap[i];
    }
    Float expected_rz_new = InitialRzNew;
    for(int vertex = 0; vertex < Vertices; ++vertex)
    {
        const int lane = real_to_part[vertex];
        for(int component = 0; component < 3; ++component)
        {
            const int i = vertex * 3 + component;
            if(lane >= 0)
            {
                expected_z[i] =
                    expected_r[i] / (Float{2 + component} + Float{0.125} * lane);
            }
            else
            {
                expected_z[i] = 0;
                for(int j = 0; j < 3; ++j)
                    expected_z[i] +=
                        diag_inv[vertex](component, j) * expected_r[vertex * 3 + j];
            }
            expected_rz_new += expected_r[i] * expected_z[i];
        }
    }

    cuda_tool::DeviceDenseVector<Float> d_x, d_p, d_r, d_Ap, d_z;
    for(auto* vector : {&d_x, &d_p, &d_r, &d_Ap, &d_z})
        vector->resize(Scalars);
    d_x.buffer_view().copy_from(x.data());
    d_p.buffer_view().copy_from(p.data());
    d_r.buffer_view().copy_from(r.data());
    d_Ap.buffer_view().copy_from(Ap.data());
    d_z.buffer_view().fill(Float{-100});
    cuda_tool::DeviceVar<Float>  d_rz{Rz};
    cuda_tool::DeviceVar<Float>  d_pAp{PAp};
    cuda_tool::DeviceVar<Float>  d_rz_new{InitialRzNew};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    auto fused_step = [&]
    {
        engine.fused_pcg_update_apply_dot(d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          nullptr,
                                          d_diag_inv.cview());
    };
    fused_step();

    std::vector<Float> actual_x(Scalars), actual_r(Scalars), actual_z(Scalars);
    d_x.buffer_view().copy_to(actual_x.data());
    d_r.buffer_view().copy_to(actual_r.data());
    d_z.buffer_view().copy_to(actual_z.data());
    require_near(actual_x, expected_x, 1e-12);
    require_near(actual_r, expected_r, 1e-12);
    require_near(actual_z, expected_z);
    for(int vertex : unpartitioned)
        for(int component = 0; component < 3; ++component)
            REQUIRE(actual_z[vertex * 3 + component]
                    == Catch::Approx(expected_z[vertex * 3 + component]).epsilon(1e-14));
    REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).margin(1e-4));

    SECTION("converged status is a full no-op")
    {
        d_x.buffer_view().fill(Float{7});
        d_r.buffer_view().fill(Float{11});
        d_z.buffer_view().fill(Float{13});
        d_rz_new    = Float{17};
        d_converged = IndexT{1};
        fused_step();

        d_x.buffer_view().copy_to(actual_x.data());
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        require_near(actual_x, std::vector<Float>(Scalars, Float{7}), Float{0});
        require_near(actual_r, std::vector<Float>(Scalars, Float{11}), Float{0});
        require_near(actual_z, std::vector<Float>(Scalars, Float{13}), Float{0});
        REQUIRE(static_cast<Float>(d_rz_new) == Float{17});
    }
}

namespace
{
// A chain of four clusters, so the hierarchy has coarse levels whose
// corrections reach z, and the vectors of a fused PCG step on it. The host
// copy of r follows the device: every fused step subtracts Alpha * Ap.
struct CoarseLevelChain
{
    static constexpr int   Vertices = 4 * MASPreconditionerEngine::BANKSIZE;
    static constexpr int   Scalars  = Vertices * 3;
    static constexpr Float Rz       = 3.0;
    static constexpr Float PAp      = 2.0;
    static constexpr Float Alpha    = Rz / PAp;

    std::vector<unsigned int> neighbor_starts = std::vector<unsigned int>(Vertices);
    std::vector<unsigned int> neighbor_counts = std::vector<unsigned int>(Vertices);
    std::vector<unsigned int> neighbor_list;
    std::vector<int>          identity = std::vector<int>(Vertices);

    cuda_tool::DeviceBuffer<Matrix3x3> d_values;
    cuda_tool::DeviceBuffer<int>       d_rows;
    cuda_tool::DeviceBuffer<int>       d_cols;
    cuda_tool::DeviceDenseVector<Float> d_x, d_p, d_r, d_Ap, d_z, d_r_ref, d_z_ref;
    cuda_tool::DeviceVar<Float>  d_rz{Rz};
    cuda_tool::DeviceVar<Float>  d_pAp{PAp};
    cuda_tool::DeviceVar<Float>  d_rz_new{Float{0}};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    std::vector<Float> r  = std::vector<Float>(Scalars);
    std::vector<Float> Ap = std::vector<Float>(Scalars);

    CoarseLevelChain()
    {
        for(int vertex = 0; vertex < Vertices; ++vertex)
        {
            neighbor_starts[vertex] = static_cast<unsigned int>(neighbor_list.size());
            if(vertex > 0)
                neighbor_list.push_back(vertex - 1);
            if(vertex + 1 < Vertices)
                neighbor_list.push_back(vertex + 1);
            neighbor_counts[vertex] = static_cast<unsigned int>(neighbor_list.size())
                                      - neighbor_starts[vertex];
        }
        std::iota(identity.begin(), identity.end(), 0);

        std::vector<Matrix3x3> values;
        std::vector<int>       rows;
        std::vector<int>       cols;
        for(int vertex = 0; vertex < Vertices; ++vertex)
        {
            Matrix3x3 diagonal = Matrix3x3::Zero();
            for(int component = 0; component < 3; ++component)
                diagonal(component, component) =
                    Float{4 + component} + Float{0.0625} * vertex;
            values.push_back(diagonal);
            rows.push_back(vertex);
            cols.push_back(vertex);
            if(vertex + 1 < Vertices)
            {
                values.push_back(Float{-0.5} * Matrix3x3::Identity());
                rows.push_back(vertex);
                cols.push_back(vertex + 1);
            }
        }
        d_values.resize(values.size());
        d_rows.resize(rows.size());
        d_cols.resize(cols.size());
        d_values.view().copy_from(values.data());
        d_rows.view().copy_from(rows.data());
        d_cols.view().copy_from(cols.data());

        for(auto* vector : {&d_x, &d_p, &d_r, &d_Ap, &d_z, &d_r_ref, &d_z_ref})
            vector->resize(Scalars);
        std::vector<Float> x(Scalars);
        for(int i = 0; i < Scalars; ++i)
        {
            x[i]  = Float{0.01 * (i + 1)};
            r[i]  = Float{0.4 + 0.003 * i};
            Ap[i] = Float{0.005 * (i % 5 + 1)};
        }
        d_x.buffer_view().copy_from(x.data());
        d_r.buffer_view().copy_from(r.data());
        d_Ap.buffer_view().copy_from(Ap.data());
    }

    void make_engine(MASPreconditionerEngine& engine, int active_levels)
    {
        engine.init_neighbor(Vertices,
                             static_cast<int>(neighbor_list.size()),
                             Vertices,
                             span<const unsigned int>{neighbor_list},
                             span<const unsigned int>{neighbor_starts},
                             span<const unsigned int>{neighbor_counts},
                             span<const int>{identity},
                             span<const int>{identity});
        engine.init_matrix();
        engine.set_active_level_num(active_levels);
        engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);
    }

    // Uploads a different direction every step, so stale coarse residuals of
    // the previous step would show up in z, and applies the step's residual
    // update to the host copy of r.
    void begin_step(int step)
    {
        std::vector<Float> p(Scalars);
        for(int i = 0; i < Scalars; ++i)
            p[i] = Float{0.02 * ((i + 3 * step) % 7 + 1) * (step % 2 ? -1 : 1)};
        d_p.buffer_view().copy_from(p.data());
        for(int i = 0; i < Scalars; ++i)
            r[i] -= Alpha * Ap[i];
        d_rz_new = Float{0};
    }

    void fused_step(MASPreconditionerEngine& engine, cudaStream_t stream)
    {
        engine.fused_pcg_update_apply_dot(d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          stream);
    }

    // Checks the step's r, z and r^T z against apply() of `reference`, which
    // always clears the coarse levels, and returns how much the coarse levels
    // changed z (against `fine`, which uses the finest level only).
    Float check_step(int step, MASPreconditionerEngine& reference, MASPreconditionerEngine& fine)
    {
        d_r_ref.buffer_view().copy_from(r.data());
        reference.apply(d_r_ref.cview(), d_z_ref.view(), d_converged.view(), nullptr);

        std::vector<Float> actual_r(Scalars), actual_z(Scalars), expected_z(Scalars);
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        d_z_ref.buffer_view().copy_to(expected_z.data());
        require_near(actual_r, r, 1e-12);
        Float expected_rz_new = 0;
        for(int i = 0; i < Scalars; ++i)
        {
            INFO("step " << step << " scalar " << i);
            REQUIRE(actual_z[i] == Catch::Approx(expected_z[i]).epsilon(1e-5).margin(1e-6));
            expected_rz_new += r[i] * expected_z[i];
        }
        REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).epsilon(1e-5));

        fine.apply(d_r_ref.cview(), d_z_ref.view(), d_converged.view(), nullptr);
        std::vector<Float> fine_z(Scalars);
        d_z_ref.buffer_view().copy_to(fine_z.data());
        Float coarse_effect = 0;
        for(int i = 0; i < Scalars; ++i)
            coarse_effect =
                std::max(coarse_effect, std::abs(expected_z[i] - fine_z[i]));
        return coarse_effect;
    }
};
}  // namespace

TEST_CASE("MAS fused PCG step without a partitioned vertex serves every vertex with the diagonal fallback",
          "[build_solve_focused][mas][fused_pcg]")
{
    // Every FEM vertex unpartitioned (part_map_size 0): there is no hierarchy
    // to restrict into, and the step must not launch the restriction with an
    // empty grid.
    constexpr int Vertices = 5;
    constexpr int Scalars  = Vertices * 3;

    std::vector<int>          part_to_real;
    std::vector<int>          real_to_part(Vertices, -1);
    std::vector<unsigned int> neighbor_starts(Vertices, 0);
    std::vector<unsigned int> neighbor_counts(Vertices, 0);
    std::vector<unsigned int> neighbor_list;

    MASPreconditionerEngine engine;
    engine.init_neighbor(Vertices,
                         0,
                         0,
                         span<const unsigned int>{neighbor_list},
                         span<const unsigned int>{neighbor_starts},
                         span<const unsigned int>{neighbor_counts},
                         span<const int>{part_to_real},
                         span<const int>{real_to_part});
    engine.init_matrix();
    engine.set_active_level_num(1);

    std::vector<Matrix3x3> values(Vertices, Matrix3x3::Identity());
    std::vector<int>       rows(Vertices);
    for(int vertex = 0; vertex < Vertices; ++vertex)
        rows[vertex] = vertex;
    cuda_tool::DeviceBuffer<Matrix3x3> d_values(Vertices);
    cuda_tool::DeviceBuffer<int>       d_rows(Vertices);
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_rows.cview(), 0);

    std::vector<Matrix3x3> diag_inv(Vertices);
    for(int vertex = 0; vertex < Vertices; ++vertex)
        diag_inv[vertex] << 2.0, 0.25, 0.125, 0.25, 3.0, 0.5, 0.125, 0.5, 4.0 + 0.1 * vertex;
    cuda_tool::DeviceBuffer<Matrix3x3> d_diag_inv(Vertices);
    d_diag_inv.view().copy_from(diag_inv.data());

    constexpr Float Rz           = 3.0;
    constexpr Float PAp          = 2.0;
    constexpr Float Alpha        = Rz / PAp;
    constexpr Float InitialRzNew = 0.75;

    std::vector<Float> x(Scalars), p(Scalars), r(Scalars), Ap(Scalars);
    std::vector<Float> expected_x(Scalars), expected_r(Scalars), expected_z(Scalars);
    for(int i = 0; i < Scalars; ++i)
    {
        x[i]          = Float{0.01 * (i + 1)};
        p[i]          = Float{0.02 * (i % 7 + 1)};
        r[i]          = Float{0.4 + 0.003 * i};
        Ap[i]         = Float{0.005 * (i % 5 + 1)};
        expected_x[i] = x[i] + Alpha * p[i];
        expected_r[i] = r[i] - Alpha * Ap[i];
    }
    Float expected_rz_new = InitialRzNew;
    for(int vertex = 0; vertex < Vertices; ++vertex)
        for(int component = 0; component < 3; ++component)
        {
            const int i   = vertex * 3 + component;
            expected_z[i] = 0;
            for(int j = 0; j < 3; ++j)
                expected_z[i] +=
                    diag_inv[vertex](component, j) * expected_r[vertex * 3 + j];
            expected_rz_new += expected_r[i] * expected_z[i];
        }

    cuda_tool::DeviceDenseVector<Float> d_x, d_p, d_r, d_Ap, d_z;
    for(auto* vector : {&d_x, &d_p, &d_r, &d_Ap, &d_z})
        vector->resize(Scalars);
    d_x.buffer_view().copy_from(x.data());
    d_p.buffer_view().copy_from(p.data());
    d_r.buffer_view().copy_from(r.data());
    d_Ap.buffer_view().copy_from(Ap.data());
    d_z.buffer_view().fill(Float{-100});
    cuda_tool::DeviceVar<Float>  d_rz{Rz};
    cuda_tool::DeviceVar<Float>  d_pAp{PAp};
    cuda_tool::DeviceVar<Float>  d_rz_new{InitialRzNew};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    REQUIRE(cudaGetLastError() == cudaSuccess);
    engine.fused_pcg_update_apply_dot(d_x.view(),
                                      d_p.cview(),
                                      d_r.view(),
                                      d_Ap.cview(),
                                      d_z.view(),
                                      d_rz.view(),
                                      d_pAp.view(),
                                      d_rz_new.view(),
                                      d_converged.view(),
                                      nullptr,
                                      d_diag_inv.cview());
    REQUIRE(cudaGetLastError() == cudaSuccess);
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

    std::vector<Float> actual_x(Scalars), actual_r(Scalars), actual_z(Scalars);
    d_x.buffer_view().copy_to(actual_x.data());
    d_r.buffer_view().copy_to(actual_r.data());
    d_z.buffer_view().copy_to(actual_z.data());
    require_near(actual_x, expected_x, 1e-12);
    require_near(actual_r, expected_r, 1e-12);
    require_near(actual_z, expected_z, 1e-12);
    REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).margin(1e-10));
}

TEST_CASE("MAS fused PCG steps reuse the coarse residual levels the local solve cleared",
          "[build_solve_focused][mas][fused_pcg]")
{
    // The fused step clears the coarse residual levels only before its first
    // use; afterwards the Schwarz local solve clears them. Repeated fused
    // steps must match apply() (which always clears) on a twin engine for
    // every step.
    CoarseLevelChain        chain;
    MASPreconditionerEngine fused_engine;
    MASPreconditionerEngine reference_engine;
    MASPreconditionerEngine fine_engine;
    chain.make_engine(fused_engine, 0);
    chain.make_engine(reference_engine, 0);
    chain.make_engine(fine_engine, 1);

    Float coarse_effect = 0;
    for(int step = 0; step < 4; ++step)
    {
        chain.begin_step(step);
        chain.fused_step(fused_engine, nullptr);
        coarse_effect =
            std::max(coarse_effect, chain.check_step(step, reference_engine, fine_engine));
    }
    // the coarse levels do change z, so the comparison above covers them
    REQUIRE(coarse_effect > Float{1e-3});
}

TEST_CASE("MAS fused PCG steps keep the coarse residual levels clear across graph replays",
          "[build_solve_focused][mas][fused_pcg]")
{
    // The fused PCG captures the fused step after an apply() of the same
    // solve (eagerly before the replay blocks, in the WHILE setup chain), so
    // the captured graph holds no clear of the coarse levels and each replay
    // relies on the local solve of the step before. Capture one step on a
    // forked branch, as the PCG runs it beside another segment, and replay it
    // four times against apply() on a twin engine; a capture before any
    // apply() records the clear instead.
    size_t nodes_after_apply = 0;
    size_t nodes_fresh       = 0;
    for(bool after_apply : {true, false})
    {
        CoarseLevelChain        chain;
        MASPreconditionerEngine captured_engine;
        MASPreconditionerEngine reference_engine;
        MASPreconditionerEngine fine_engine;
        chain.make_engine(captured_engine, 0);
        chain.make_engine(reference_engine, 0);
        chain.make_engine(fine_engine, 1);
        if(after_apply)
            captured_engine.apply(
                chain.d_r.cview(), chain.d_z.view(), chain.d_converged.view(), nullptr);

        // a blocking capture stream, as GraphCapture and GraphWhile use
        cudaStream_t stream = nullptr;
        REQUIRE(cudaStreamCreateWithFlags(&stream, cudaStreamDefault) == cudaSuccess);
        cuda_tool::SideStreams branch;
        branch.create(1);
        cudaGraph_t graph = nullptr;
        REQUIRE(cudaStreamBeginCapture(stream, cudaStreamCaptureModeThreadLocal) == cudaSuccess);
        branch.fork(stream);
        chain.fused_step(captured_engine, branch[0]);
        branch.join(stream);
        REQUIRE(cudaStreamEndCapture(stream, &graph) == cudaSuccess);
        size_t& nodes = after_apply ? nodes_after_apply : nodes_fresh;
        REQUIRE(cudaGraphGetNodes(graph, nullptr, &nodes) == cudaSuccess);
        cudaGraphExec_t exec = nullptr;
        REQUIRE(cudaGraphInstantiateWithFlags(&exec, graph, 0) == cudaSuccess);

        Float coarse_effect = 0;
        for(int step = 0; step < 4; ++step)
        {
            INFO("captured after apply(): " << after_apply);
            chain.begin_step(step);
            REQUIRE(cudaGraphLaunch(exec, stream) == cudaSuccess);
            REQUIRE(cudaStreamSynchronize(stream) == cudaSuccess);
            coarse_effect =
                std::max(coarse_effect, chain.check_step(step, reference_engine, fine_engine));
        }
        REQUIRE(coarse_effect > Float{1e-3});

        REQUIRE(cudaGraphExecDestroy(exec) == cudaSuccess);
        REQUIRE(cudaGraphDestroy(graph) == cudaSuccess);
        REQUIRE(cudaStreamDestroy(stream) == cudaSuccess);
    }
    // the graph captured after apply() lacks exactly the clear
    REQUIRE(nodes_after_apply + 1 == nodes_fresh);
}

namespace
{
// Three clusters of partitioned vertices (a chain in partition order) and 37
// unpartitioned vertices interleaved in real order, so the hierarchy has
// coarse levels and empty banks between level 0 and level 1; diagonal
// fallback inverses (full 3x3) for the unpartitioned vertices and NaN for
// the partitioned ones, which must not be read.
struct UnpartitionedGapChain
{
    static constexpr int Partitioned = 3 * MASPreconditionerEngine::BANKSIZE;
    static constexpr int Vertices    = 85;
    static constexpr int Scalars     = Vertices * 3;

    std::vector<int> part_to_real;
    std::vector<int> real_to_part = std::vector<int>(Vertices, -1);
    std::vector<int> unpart_flags = std::vector<int>(Vertices, 0);
    std::vector<unsigned int> neighbor_starts = std::vector<unsigned int>(Vertices);
    std::vector<unsigned int> neighbor_counts = std::vector<unsigned int>(Vertices);
    std::vector<unsigned int> neighbor_list;
    std::vector<Matrix3x3>    diag_inv = std::vector<Matrix3x3>(
        Vertices, Matrix3x3::Constant(std::numeric_limits<Float>::quiet_NaN()));
    std::vector<Float> r = std::vector<Float>(Scalars);

    cuda_tool::DeviceBuffer<Matrix3x3>  d_values;
    cuda_tool::DeviceBuffer<int>        d_rows, d_cols;
    cuda_tool::DeviceBuffer<Matrix3x3>  d_diag_inv;
    cuda_tool::DeviceBuffer<int>        d_flags;
    cuda_tool::DeviceDenseVector<Float> d_r;

    UnpartitionedGapChain()
    {
        for(int vertex = 0; vertex < Vertices; ++vertex)
        {
            // 37 * v mod 85 is a bijection: exactly 37 vertices are unpartitioned
            if((37 * vertex) % Vertices < Vertices - Partitioned)
            {
                unpart_flags[vertex] = 1;
                continue;
            }
            real_to_part[vertex] = static_cast<int>(part_to_real.size());
            part_to_real.push_back(vertex);
        }

        for(int vertex = 0; vertex < Vertices; ++vertex)
        {
            neighbor_starts[vertex] = static_cast<unsigned int>(neighbor_list.size());
            const int part = real_to_part[vertex];
            if(part > 0)
                neighbor_list.push_back(part_to_real[part - 1]);
            if(part >= 0 && part + 1 < Partitioned)
                neighbor_list.push_back(part_to_real[part + 1]);
            neighbor_counts[vertex] = static_cast<unsigned int>(neighbor_list.size())
                                      - neighbor_starts[vertex];
        }

        // Upper BCOO: a diagonal block for every vertex, couplings along the chain
        std::vector<Matrix3x3> values;
        std::vector<int>       rows, cols;
        for(int vertex = 0; vertex < Vertices; ++vertex)
        {
            Matrix3x3 diagonal = Matrix3x3::Zero();
            for(int component = 0; component < 3; ++component)
                diagonal(component, component) =
                    Float{4 + component} + Float{0.0625} * vertex;
            values.push_back(diagonal);
            rows.push_back(vertex);
            cols.push_back(vertex);
        }
        for(int part = 0; part + 1 < Partitioned; ++part)
        {
            const int a = part_to_real[part], b = part_to_real[part + 1];
            values.push_back(Float{-0.5} * Matrix3x3::Identity());
            rows.push_back(std::min(a, b));
            cols.push_back(std::max(a, b));
        }
        d_values.resize(values.size());
        d_rows.resize(rows.size());
        d_cols.resize(cols.size());
        d_values.view().copy_from(values.data());
        d_rows.view().copy_from(rows.data());
        d_cols.view().copy_from(cols.data());

        for(int vertex = 0; vertex < Vertices; ++vertex)
        {
            if(unpart_flags[vertex])
            {
                Matrix3x3 m;
                m << 1.0 / 3, 0.25 * vertex, -1e-3, 1e5 / (vertex + 1), 3.0,
                    0.5, 0.125, -0.7 / 7, 4.0 + 0.1 * vertex;
                diag_inv[vertex] = m;
            }
            for(int component = 0; component < 3; ++component)
                r[vertex * 3 + component] = std::sin(Float{0.37} * (vertex * 3 + component))
                                            * std::pow(Float{10}, vertex % 7 - 3);
        }
        d_diag_inv.resize(Vertices);
        d_flags.resize(Vertices);
        d_diag_inv.view().copy_from(diag_inv.data());
        d_flags.view().copy_from(unpart_flags.data());
        d_r.resize(Scalars);
        d_r.buffer_view().copy_from(r.data());
    }

    void init_engine(MASPreconditionerEngine& engine)
    {
        engine.init_neighbor(Vertices,
                             static_cast<int>(neighbor_list.size()),
                             Partitioned,
                             span<const unsigned int>{neighbor_list},
                             span<const unsigned int>{neighbor_starts},
                             span<const unsigned int>{neighbor_counts},
                             span<const int>{part_to_real},
                             span<const int>{real_to_part});
        engine.init_matrix();
    }

    void assemble(MASPreconditionerEngine& engine)
    {
        engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);
    }
};
}  // namespace

TEST_CASE("MAS apply serves the diagonal fallback of unpartitioned vertices like the separate pass",
          "[build_solve_focused][mas]")
{
    using Chain = UnpartitionedGapChain;
    Chain chain;
    REQUIRE(chain.part_to_real.size() == Chain::Partitioned);

    MASPreconditionerEngine engine;
    chain.init_engine(engine);
    chain.assemble(engine);

    cuda_tool::DeviceDenseVector<Float> d_z_fused, d_z_separate;
    for(auto* vector : {&d_z_fused, &d_z_separate})
        vector->resize(Chain::Scalars);
    d_z_fused.buffer_view().fill(Float{-77});
    d_z_separate.buffer_view().fill(Float{-77});
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    engine.apply(chain.d_r.cview(), d_z_separate.view(), d_converged.view(), nullptr);
    reference_apply_diag_inv_for_unpartitioned_kernel<<<(Chain::Vertices + 127) / 128, 128>>>(
        chain.d_r.cview(),
        d_z_separate.view(),
        chain.d_diag_inv.cview(),
        chain.d_flags.cview(),
        d_converged.data(),
        Chain::Vertices);
    engine.apply(chain.d_r.cview(),
                 d_z_fused.view(),
                 d_converged.view(),
                 nullptr,
                 chain.d_diag_inv.cview());
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

    std::vector<Float> z_fused(Chain::Scalars), z_separate(Chain::Scalars);
    d_z_fused.buffer_view().copy_to(z_fused.data());
    d_z_separate.buffer_view().copy_to(z_separate.data());
    int mismatches = 0;
    for(int i = 0; i < Chain::Scalars; ++i)
    {
        INFO("scalar " << i);
        REQUIRE(std::isfinite(z_fused[i]));
        mismatches += std::memcmp(&z_fused[i], &z_separate[i], sizeof(Float)) != 0;
    }
    CHECK(mismatches == 0);
    for(int vertex = 0; vertex < Chain::Vertices; ++vertex)
    {
        if(!chain.unpart_flags[vertex])
            continue;
        const Vector3 rv{chain.r[vertex * 3], chain.r[vertex * 3 + 1], chain.r[vertex * 3 + 2]};
        const Vector3 expected = chain.diag_inv[vertex] * rv;
        const Vector3 scale = chain.diag_inv[vertex].cwiseAbs() * rv.cwiseAbs();
        for(int component = 0; component < 3; ++component)
            REQUIRE(std::abs(z_fused[vertex * 3 + component] - expected[component])
                    <= Float{4e-15} * scale[component]);
    }

    SECTION("converged status is a full no-op")
    {
        d_z_fused.buffer_view().fill(Float{13});
        d_converged = IndexT{1};
        engine.apply(chain.d_r.cview(),
                     d_z_fused.view(),
                     d_converged.view(),
                     nullptr,
                     chain.d_diag_inv.cview());
        d_z_fused.buffer_view().copy_to(z_fused.data());
        require_near(z_fused, std::vector<Float>(Chain::Scalars, Float{13}), Float{0});
    }
}

namespace uipc::backend::cuda
{
// Fills the empty banks between level 0 and level 1 of every per-cluster and
// per-node buffer of an engine with all-one bytes (NaN in float and double)
// and checks that they still hold them.
class MASPreconditionerEngineTestAccess
{
    using Engine = MASPreconditionerEngine;

  public:
    static int empty_clusters(const Engine& e)
    {
        return e.empty_cluster_count();
    }

    static void poison_empty_banks(Engine& e)
    {
        const int     first = e.level0_cluster_count();
        const int     count = e.empty_cluster_count();
        constexpr int Bank  = Engine::BANKSIZE;
        REQUIRE(cudaMemset(e.cluster_hessians.data() + first, 0xff, count * sizeof(Engine::ClusterMatrixSym))
                == cudaSuccess);
        REQUIRE(cudaMemset(e.cluster_inverses.data() + first, 0xff, count * sizeof(Engine::ClusterMatrixSymF))
                == cudaSuccess);
        REQUIRE(cudaMemset(e.multi_level_R.data() + first * Bank,
                           0xff,
                           count * Bank * sizeof(Eigen::Vector3f))
                == cudaSuccess);
        REQUIRE(cudaMemset(e.multi_level_Z.data() + first * Bank, 0xff, count * Bank * sizeof(float3))
                == cudaSuccess);
    }

    static bool empty_banks_poisoned(const Engine& e)
    {
        const int     first    = e.level0_cluster_count();
        const int     count    = e.empty_cluster_count();
        constexpr int Bank     = Engine::BANKSIZE;
        auto          all_ones = [](const void* device, size_t bytes)
        {
            std::vector<unsigned char> host(bytes);
            REQUIRE(cudaMemcpy(host.data(), device, bytes, cudaMemcpyDeviceToHost) == cudaSuccess);
            return std::all_of(host.begin(),
                               host.end(),
                               [](unsigned char b) { return b == 0xff; });
        };
        return all_ones(e.cluster_hessians.data() + first,
                        count * sizeof(Engine::ClusterMatrixSym))
               && all_ones(e.cluster_inverses.data() + first,
                           count * sizeof(Engine::ClusterMatrixSymF))
               && all_ones(e.multi_level_R.data() + first * Bank,
                           count * Bank * sizeof(Eigen::Vector3f))
               && all_ones(e.multi_level_Z.data() + first * Bank,
                           count * Bank * sizeof(float3));
    }

    static int level_num(const Engine& e) { return e.m_level_num; }

    // The stream prepare_preconditioner() clears the Hessians on, ordered
    // after the work enqueued on the default stream so far (as it orders it).
    static cudaStream_t hessian_clear_stream(Engine& e)
    {
        return e.m_hessian_clear_stream.fork();
    }

    static int level1_begin(const Engine& e) { return e.m_level1_begin; }
    static int total_clusters(const Engine& e)
    {
        return e.m_total_num_clusters;
    }
    static int skip_begin(const Engine& e) { return e.level0_cluster_count(); }
    static int skip_count(const Engine& e) { return e.empty_cluster_count(); }
    static auto& inverses(Engine& e) { return e.cluster_inverses; }
    static auto& residuals(Engine& e) { return e.multi_level_R; }
    static auto& solutions(Engine& e) { return e.multi_level_Z; }
    static auto& going_next_buffer(Engine& e) { return e.going_next; }
    static auto& level_sizes(Engine& e) { return e.level_sizes; }
    static auto& prefix_original(Engine& e) { return e.prefix_original; }
    static auto& fine_connect_masks(Engine& e) { return e.fine_connect_masks; }
    static auto& part_to_real(Engine& e) { return e.part_to_real; }
    static auto& real_to_part(Engine& e) { return e.real_to_part; }
    static auto& coarse_tables(Engine& e) { return e.coarse_tables; }
    static int total_map_nodes(const Engine& e) { return e.m_total_map_nodes; }
    static int total_nodes(const Engine& e) { return e.m_total_nodes; }

    // every cluster's 136 blocks of 3x3 doubles (column-major), as stored
    static std::vector<double> cluster_hessians(const Engine& e)
    {
        std::vector<double> host(e.cluster_hessians.size() * Engine::SYM_BLOCK_COUNT * 9);
        REQUIRE(cudaMemcpy(host.data(), e.cluster_hessians.data(), host.size() * sizeof(double), cudaMemcpyDeviceToHost)
                == cudaSuccess);
        return host;
    }

    static std::vector<int> going_next(const Engine& e)
    {
        std::vector<int> host(e.going_next.size());
        REQUIRE(cudaMemcpy(host.data(), e.going_next.data(), host.size() * sizeof(int), cudaMemcpyDeviceToHost)
                == cudaSuccess);
        return host;
    }
};
}  // namespace uipc::backend::cuda

TEST_CASE("MAS passes neither read nor write the empty banks between level 0 and level 1",
          "[build_solve_focused][mas]")
{
    using Chain  = UnpartitionedGapChain;
    using Access = MASPreconditionerEngineTestAccess;
    Chain chain;

    // NaN in every empty bank of one engine: z must equal a clean twin's, and
    // the NaN must still be there afterwards.
    MASPreconditionerEngine poisoned, clean;
    chain.init_engine(poisoned);
    chain.init_engine(clean);
    REQUIRE(Access::empty_clusters(poisoned) == 3);
    Access::poison_empty_banks(poisoned);
    chain.assemble(poisoned);
    chain.assemble(clean);

    cuda_tool::DeviceDenseVector<Float> d_z_poisoned, d_z_clean;
    for(auto* vector : {&d_z_poisoned, &d_z_clean})
        vector->resize(Chain::Scalars);
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};
    std::vector<Float> z_poisoned(Chain::Scalars), z_clean(Chain::Scalars);
    auto               apply_both = [&]
    {
        poisoned.apply(chain.d_r.cview(),
                       d_z_poisoned.view(),
                       d_converged.view(),
                       nullptr,
                       chain.d_diag_inv.cview());
        clean.apply(chain.d_r.cview(),
                    d_z_clean.view(),
                    d_converged.view(),
                    nullptr,
                    chain.d_diag_inv.cview());
        d_z_poisoned.buffer_view().copy_to(z_poisoned.data());
        d_z_clean.buffer_view().copy_to(z_clean.data());
    };

    // Fine level only: z does not depend on the order of the coarse atomics,
    // so the two engines agree bit for bit.
    poisoned.set_active_level_num(1);
    clean.set_active_level_num(1);
    apply_both();
    for(int i = 0; i < Chain::Scalars; ++i)
    {
        INFO("scalar " << i);
        REQUIRE(std::isfinite(z_poisoned[i]));
        REQUIRE(std::memcmp(&z_poisoned[i], &z_clean[i], sizeof(Float)) == 0);
    }

    // Full hierarchy, twice (the second apply finds the coarse levels the
    // first one cleared): equal up to the order of the coarse atomics.
    poisoned.set_active_level_num(0);
    clean.set_active_level_num(0);
    for(int round = 0; round < 2; ++round)
    {
        apply_both();
        for(int i = 0; i < Chain::Scalars; ++i)
        {
            INFO("round " << round << " scalar " << i);
            REQUIRE(std::isfinite(z_poisoned[i]));
            REQUIRE(z_poisoned[i] == Catch::Approx(z_clean[i]).epsilon(1e-5).margin(1e-9));
        }
    }

    // a fused PCG step as well
    cuda_tool::DeviceDenseVector<Float> d_x, d_p, d_r, d_Ap;
    for(auto* vector : {&d_x, &d_p, &d_r, &d_Ap})
        vector->resize(Chain::Scalars);
    d_x.buffer_view().fill(Float{0});
    d_p.buffer_view().fill(Float{0.25});
    d_r.buffer_view().copy_from(chain.r.data());
    d_Ap.buffer_view().fill(Float{0.125});
    cuda_tool::DeviceVar<Float> d_rz{Float{3}}, d_pAp{Float{2}}, d_rz_new{Float{0}};
    poisoned.fused_pcg_update_apply_dot(d_x.view(),
                                        d_p.cview(),
                                        d_r.view(),
                                        d_Ap.cview(),
                                        d_z_poisoned.view(),
                                        d_rz.view(),
                                        d_pAp.view(),
                                        d_rz_new.view(),
                                        d_converged.view(),
                                        nullptr,
                                        chain.d_diag_inv.cview());
    d_z_poisoned.buffer_view().copy_to(z_poisoned.data());
    for(int i = 0; i < Chain::Scalars; ++i)
        REQUIRE(std::isfinite(z_poisoned[i]));
    REQUIRE(std::isfinite(static_cast<Float>(d_rz_new)));

    REQUIRE(Access::empty_banks_poisoned(poisoned));
}

namespace
{
// Keeps its stream busy for `ns` nanoseconds.
__global__ void mas_test_hold_stream_kernel(unsigned long long ns)
{
    unsigned long long start, now;
    asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(start));
    do
    {
        __nanosleep(1000);
        asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(now));
    } while(now - start < ns);
}
}  // namespace

TEST_CASE("MAS scatter adds every fine block to each coarse block once", "[build_solve_focused][mas]")
{
    using Access       = MASPreconditionerEngineTestAccess;
    constexpr int Bank = MASPreconditionerEngine::BANKSIZE;

    // A 5000-vertex chain with irregular breaks (five levels) and long-range
    // couplings, so that entries cross banks, merge at several levels, and
    // land on coarse diagonal blocks (both nodes in one coarse node) as well
    // as off-diagonal ones. Block entries are multiples of 1/16, so every sum
    // below is exact in double and the order of the atomics cannot matter.
    constexpr int             Vertices = 5000;
    std::vector<unsigned int> starts(Vertices), counts(Vertices), list;
    std::vector<int>          identity(Vertices);
    auto                      linked = [](int v)
    { return (static_cast<unsigned int>(v) * 2654435761u) % 9u != 0u; };
    for(int v = 0; v < Vertices; ++v)
    {
        starts[v] = static_cast<unsigned int>(list.size());
        if(v > 0 && linked(v - 1))
            list.push_back(v - 1);
        if(v + 1 < Vertices && linked(v))
            list.push_back(v + 1);
        counts[v] = static_cast<unsigned int>(list.size()) - starts[v];
    }
    std::iota(identity.begin(), identity.end(), 0);

    std::vector<Matrix3x3> values;
    std::vector<int>       rows, cols;
    int                    seed  = 1;
    auto                   block = [&]
    {
        Matrix3x3 m;
        for(int k = 0; k < 9; ++k)
        {
            seed            = (seed * 7919 + 13) % 8191;
            m(k % 3, k / 3) = Float(seed % 129 - 64) / 16;
        }
        return m;
    };
    auto add = [&](int r, int c)
    {
        rows.push_back(r);
        cols.push_back(c);
        values.push_back(block());
    };
    for(int v = 0; v < Vertices; ++v)
    {
        add(v, v);
        if(v + 1 < Vertices && linked(v))
            add(v, v + 1);
        if(v % 5 == 0 && v + 97 < Vertices)
            add(v, v + 97);
        if(v % 11 == 0 && v + 700 < Vertices)
            add(v, v + 700);
    }
    cuda_tool::DeviceBuffer<Matrix3x3> d_values;
    cuda_tool::DeviceBuffer<int>       d_rows, d_cols;
    d_values.resize(values.size());
    d_rows.resize(rows.size());
    d_cols.resize(cols.size());
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    d_cols.view().copy_from(cols.data());

    MASPreconditionerEngine engine;
    engine.init_neighbor(Vertices,
                         static_cast<int>(list.size()),
                         Vertices,
                         span<const unsigned int>{list},
                         span<const unsigned int>{starts},
                         span<const unsigned int>{counts},
                         span<const int>{identity},
                         span<const int>{identity});
    engine.init_matrix();
    const int levels = Access::level_num(engine);
    REQUIRE(levels == 5);
    const std::vector<int> next = Access::going_next(engine);

    // Galerkin reference: every block (i, j) of the full symmetric matrix
    // (H_ij for i <= j, H_ji^T otherwise) goes to block (I, J) of each level
    // whose ancestors I <= J share a bank; level 0 holds the blocks of the
    // banks themselves.
    struct Key
    {
        int  I, J;
        bool operator<(const Key& o) const
        {
            return I < o.I || (I == o.I && J < o.J);
        }
    };
    std::map<Key, Matrix3x3> expected;
    auto                     deposit = [&](int I, int J, const Matrix3x3& m)
    {
        auto [it, fresh] = expected.try_emplace(Key{I, J}, Matrix3x3::Zero());
        it->second += m;
    };
    for(size_t e = 0; e < values.size(); ++e)
    {
        const int r = rows[e], c = cols[e];
        for(int orient = 0; orient < (r == c ? 1 : 2); ++orient)
        {
            int i = orient ? c : r, j = orient ? r : c;
            const Matrix3x3 m = orient ? Matrix3x3(values[e].transpose()) : values[e];
            if(i / Bank == j / Bank && i <= j)
                deposit(i, j, m);
            for(int l = 1; l < levels; ++l)
            {
                i = next[i];
                j = next[j];
                if(i / Bank == j / Bank && i <= j)
                    deposit(i, j, m);
            }
        }
    }

    // Rounds 1 and 2 clear the Hessians by prepare_preconditioner() on its
    // side stream (round 0 sizes the buffers, so its prepare does nothing);
    // in round 2 that stream is held back for 50 ms first, so the scatter
    // and inversion must wait for the clear. Round 3 clears inline again.
    for(int round = 0; round < 4; ++round)
    {
        if(round == 2)
            mas_test_hold_stream_kernel<<<1, 1, 0, Access::hessian_clear_stream(engine)>>>(
                50'000'000ull);
        if(round < 3)
            engine.prepare_preconditioner();
        engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
        const std::vector<double> hess     = Access::cluster_hessians(engine);
        const int                 clusters = static_cast<int>(
            hess.size() / (MASPreconditionerEngine::SYM_BLOCK_COUNT * 9));
        auto stored = [&](int I, int J, int a, int b)
        {
            const int cluster = I / Bank;
            const int si      = MASPreconditionerEngine::BANKSIZE * (I % Bank)
                           - (I % Bank) * (I % Bank + 1) / 2 + J % Bank;
            return hess[(static_cast<size_t>(cluster) * MASPreconditionerEngine::SYM_BLOCK_COUNT + si) * 9
                        + b * 3 + a];
        };
        int coarse_diagonal = 0, coarse_off_diagonal = 0;
        for(const auto& [key, m] : expected)
        {
            REQUIRE(key.I / Bank < clusters);
            const bool coarse = key.I >= Vertices;
            coarse_diagonal += coarse && key.I == key.J;
            coarse_off_diagonal += coarse && key.I != key.J;
            for(int a = 0; a < 3; ++a)
                for(int b = 0; b < 3; ++b)
                {
                    // the inversion reads only the upper triangle of the
                    // coarse diagonal blocks
                    if(coarse && key.I == key.J && a > b)
                        continue;
                    INFO("round " << round << " block (" << key.I << ", " << key.J
                                  << ") entry (" << a << ", " << b << ")");
                    REQUIRE(stored(key.I, key.J, a, b) == m(a, b));
                }
        }
        REQUIRE(coarse_diagonal > 100);
        REQUIRE(coarse_off_diagonal > 100);

        // no other coarse entry received anything
        std::vector<char> covered(hess.size(), 0);
        for(const auto& [key, m] : expected)
        {
            const int si = Bank * (key.I % Bank)
                           - (key.I % Bank) * (key.I % Bank + 1) / 2 + key.J % Bank;
            const size_t at =
                (static_cast<size_t>(key.I / Bank) * MASPreconditionerEngine::SYM_BLOCK_COUNT + si)
                * 9;
            std::fill(covered.begin() + at, covered.begin() + at + 9, 1);
        }
        const size_t coarse_begin = static_cast<size_t>((Vertices + Bank - 1) / Bank)
                                    * MASPreconditionerEngine::SYM_BLOCK_COUNT * 9;
        for(size_t k = coarse_begin; k < hess.size(); ++k)
        {
            INFO("round " << round << " stored double " << k);
            if(!covered[k])
                REQUIRE(hess[k] == 0.0);
        }
    }
}

namespace
{
// The Schwarz local solve as it was before the inverse block was read ahead
// of the barrier: the bitwise reference.
constexpr int SchwarzBank = MASPreconditionerEngine::BANKSIZE;

// every lane of the group reaches the barriers in the reference kernels
__device__ __forceinline__ unsigned int reference_mas_logical_group_mask()
{
    constexpr unsigned int GROUP_MASK = (1U << SchwarzBank) - 1U;
    const unsigned int     lane       = threadIdx.x & 31;
    const unsigned int     base       = (lane / SchwarzBank) * SchwarzBank;
    return GROUP_MASK << base;
}

__host__ __device__ int reference_sym_index(int row, int col)
{
    return SchwarzBank * row - row * (row + 1) / 2 + col;
}

__global__ void reference_schwarz_local_solve_kernel(
    cuda_tool::CBufferView<MASPreconditionerEngine::ClusterMatrixSymF> cluster_inv,
    cuda_tool::BufferView<Eigen::Vector3f> multi_lr,
    cuda_tool::BufferView<float3>          multi_lz,
    cuda_tool::CDense<IndexT>              converged,
    int                                    coarse_begin,
    int                                    skip_begin,
    int                                    skip_count,
    int                                    N)
{
    using namespace cuda_tool;
    constexpr int BANKSIZE = SchwarzBank;

    if(*converged != 0)
        return;
    constexpr int cluster_stride = BANKSIZE * BANKSIZE;

    const int g = blockIdx.x;
    int idx = (g < skip_begin ? g : g + skip_count) * cluster_stride + threadIdx.x;
    if(idx >= N)
        return;

    int cluster_id = idx / cluster_stride;
    int local_row  = (idx % cluster_stride) / BANKSIZE;
    int local_col  = (idx % cluster_stride) % BANKSIZE;

    int vert_row = cluster_id * BANKSIZE + local_row;
    int vert_col = cluster_id * BANKSIZE + local_col;

    __shared__ Eigen::Vector3f s_R_raw[BANKSIZE];
    auto                       s_R = make_dense_1d(s_R_raw);
    if(threadIdx.x < BANKSIZE)
        s_R(threadIdx.x) = multi_lr(vert_col);
    __syncthreads();

    if(threadIdx.x < BANKSIZE && vert_col >= coarse_begin)
        multi_lr(vert_col) = Eigen::Vector3f::Zero();

    Eigen::Vector3f result;
    if(vert_col >= vert_row)
    {
        int si = reference_sym_index(local_row, local_col);
        result = cluster_inv(cluster_id).M[si] * s_R(local_col);
    }
    else
    {
        int si = reference_sym_index(local_col, local_row);
        result = cluster_inv(cluster_id).M[si].transpose() * s_R(local_col);
    }

    using WarpReduceF = cub::WarpReduce<float, BANKSIZE>;
    __shared__ typename WarpReduceF::TempStorage temp_reduce_f[BANKSIZE];
    int                logical_warp = threadIdx.x / BANKSIZE;
    const unsigned int group_mask   = reference_mas_logical_group_mask();

    result[0] = WarpReduceF(temp_reduce_f[logical_warp]).Sum(result[0]);
    __syncwarp(group_mask);
    result[1] = WarpReduceF(temp_reduce_f[logical_warp]).Sum(result[1]);
    __syncwarp(group_mask);
    result[2] = WarpReduceF(temp_reduce_f[logical_warp]).Sum(result[2]);

    if((threadIdx.x % BANKSIZE) == 0)
        multi_lz(vert_row) = float3{result[0], result[1], result[2]};
}
}  // namespace

TEST_CASE("MAS Schwarz local solve keeps the bits of the kernel that read the inverse after the barrier",
          "[build_solve_focused][mas]")
{
    using Access = MASPreconditionerEngineTestAccess;
    using SymF   = MASPreconditionerEngine::ClusterMatrixSymF;

    // the five-level chain of the scatter test, and the gap chain (empty
    // banks between level 0 and level 1)
    for(int setup = 0; setup < 2; ++setup)
    {
        CAPTURE(setup);
        MASPreconditionerEngine engine;
        UnpartitionedGapChain   gap;
        if(setup == 0)
        {
            constexpr int             Vertices = 5000;
            std::vector<unsigned int> starts(Vertices), counts(Vertices), list;
            std::vector<int>          identity(Vertices);
            auto                      linked = [](int v)
            { return (static_cast<unsigned int>(v) * 2654435761u) % 9u != 0u; };
            for(int v = 0; v < Vertices; ++v)
            {
                starts[v] = static_cast<unsigned int>(list.size());
                if(v > 0 && linked(v - 1))
                    list.push_back(v - 1);
                if(v + 1 < Vertices && linked(v))
                    list.push_back(v + 1);
                counts[v] = static_cast<unsigned int>(list.size()) - starts[v];
            }
            std::iota(identity.begin(), identity.end(), 0);
            engine.init_neighbor(Vertices,
                                 static_cast<int>(list.size()),
                                 Vertices,
                                 span<const unsigned int>{list},
                                 span<const unsigned int>{starts},
                                 span<const unsigned int>{counts},
                                 span<const int>{identity},
                                 span<const int>{identity});
            engine.init_matrix();
            // sizes the cluster and residual buffers
            cuda_tool::DeviceBuffer<Matrix3x3> d_values(1);
            cuda_tool::DeviceBuffer<int>       d_rows(1), d_cols(1);
            const Matrix3x3                    one  = Matrix3x3::Identity();
            const int                          zero = 0;
            d_values.view().copy_from(&one);
            d_rows.view().copy_from(&zero);
            d_cols.view().copy_from(&zero);
            engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);
        }
        else
        {
            gap.init_engine(engine);
            REQUIRE(Access::skip_count(engine) > 0);
            gap.assemble(engine);
        }
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

        // random inverses and residuals (any floats: only the arithmetic and
        // the stores are compared), solutions poisoned with the same bytes
        auto&                                 inv = Access::inverses(engine);
        auto&                                 R   = Access::residuals(engine);
        auto&                                 Z   = Access::solutions(engine);
        std::mt19937                          rng(7 + setup);
        std::uniform_real_distribution<float> u(-2.0f, 2.0f);
        std::vector<SymF>                     h_inv(inv.size());
        for(auto& m : h_inv)
            for(auto& b : m.M)
                for(int k = 0; k < 9; ++k)
                    b.data()[k] = u(rng) * std::pow(10.0f, float(int(rng() % 7) - 3));
        std::vector<Eigen::Vector3f> h_R(R.size());
        for(auto& v : h_R)
            v = Eigen::Vector3f(u(rng), u(rng), u(rng));
        inv.view().copy_from(h_inv.data());

        cuda_tool::DeviceBuffer<Eigen::Vector3f> ref_R(R.size());
        cuda_tool::DeviceBuffer<float3>          ref_Z(Z.size());
        R.view().copy_from(h_R.data());
        ref_R.view().copy_from(h_R.data());
        REQUIRE(cudaMemset(Z.data(), 0xab, Z.size() * sizeof(float3)) == cudaSuccess);
        REQUIRE(cudaMemset(ref_Z.data(), 0xab, ref_Z.size() * sizeof(float3)) == cudaSuccess);

        cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};
        const int N          = Access::total_clusters(engine) * SchwarzBank;
        const int skip_begin = Access::skip_begin(engine);
        const int skip_count = Access::skip_count(engine);
        const int blocks = Access::total_clusters(engine) / SchwarzBank - skip_count;
        reference_schwarz_local_solve_kernel<<<blocks, MASPreconditionerEngine::DEFAULT_BLOCKSIZE>>>(
            inv.cview(),
            ref_R.view(),
            ref_Z.view(),
            d_converged.cviewer(),
            Access::level1_begin(engine),
            skip_begin,
            skip_count,
            N);
        engine.schwarz_local_solve(d_converged.view(), nullptr);
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

        std::vector<float3>          z_new(Z.size()), z_ref(Z.size());
        std::vector<Eigen::Vector3f> r_new(R.size()), r_ref(R.size());
        Z.view().copy_to(z_new.data());
        ref_Z.view().copy_to(z_ref.data());
        R.view().copy_to(r_new.data());
        ref_R.view().copy_to(r_ref.data());
        REQUIRE(blocks > 0);
        CHECK(std::memcmp(z_new.data(), z_ref.data(), z_new.size() * sizeof(float3)) == 0);
        CHECK(std::memcmp(r_new.data(), r_ref.data(), r_new.size() * sizeof(Eigen::Vector3f))
              == 0);
        // the solve wrote solutions and cleared coarse residuals
        float3 poison;
        std::memset(&poison, 0xab, sizeof(poison));
        int written = 0, cleared = 0;
        for(const auto& z : z_new)
            written += std::memcmp(&z, &poison, sizeof(float3)) != 0;
        for(SizeT k = 0; k < r_new.size(); ++k)
            cleared += r_new[k].isZero() && !h_R[k].isZero();
        CHECK(written > 0);
        CHECK(cleared > 0);
    }
}

namespace
{
// The restriction and the prolongation with r^T z of the fused PCG step as
// they were before their operand loads were moved ahead of the stores and
// the restriction went to 64-thread blocks: the reference.
constexpr int BANKSIZE          = MASPreconditionerEngine::BANKSIZE;
constexpr int DEFAULT_BLOCKSIZE = MASPreconditionerEngine::DEFAULT_BLOCKSIZE;
constexpr int DEFAULT_WARPNUM   = MASPreconditionerEngine::DEFAULT_WARPNUM;
using Int2                      = MASPreconditionerEngine::Int2;
using LevelTable                = MASPreconditionerEngine::LevelTable;

__host__ __device__ unsigned int lanemask_lt(int lane_id)
{
    return (1U << lane_id) - 1;
}

__device__ __forceinline__ unsigned int mas_logical_group_mask()
{
    return reference_mas_logical_group_mask();
}

template <bool UpdateXR>
__global__ void former_build_multi_level_R_kernel(
    cuda_tool::DenseVectorView<Float>      X_view,
    cuda_tool::CDenseVectorView<Float>     P_view,
    cuda_tool::DenseVectorView<Float>      R_write_view,
    cuda_tool::CDenseVectorView<Float>     R_view,
    cuda_tool::CDenseVectorView<Float>     Ap_view,
    cuda_tool::CDense<Float>               rz,
    cuda_tool::CDense<Float>               pAp,
    cuda_tool::BufferView<Eigen::Vector3f> multi_lr,
    cuda_tool::CBufferView<int>            going_next,
    cuda_tool::CBufferView<Int2>           level_size,
    cuda_tool::CBufferView<int>            prefix_orig,
    cuda_tool::CBufferView<unsigned int>   fine_conn,
    cuda_tool::CBufferView<int>            part_to_real,
    cuda_tool::CDense<IndexT>              converged,
    int                                    level_num,
    int                                    N)
{
    using namespace cuda_tool;

    if(*converged != 0)
        return;
    int pdx = blockIdx.x * blockDim.x + threadIdx.x;

    bool in_range = (pdx < N);
    int  idx      = in_range ? part_to_real(pdx) : -1;
    bool is_valid = (idx >= 0);

    Eigen::Vector3f r = Eigen::Vector3f::Zero();
    if(is_valid)
    {
        auto r_read = R_view.segment<3>(3 * idx);
        if constexpr(UpdateXR)
        {
            const Float alpha = *rz / *pAp;
            auto        x     = X_view.segment<3>(3 * idx);
            auto        p     = P_view.segment<3>(3 * idx);
            auto        r_out = R_write_view.segment<3>(3 * idx);
            auto        Ap    = Ap_view.segment<3>(3 * idx);
#pragma unroll
            for(int component = 0; component < 3; ++component)
            {
                x(component) += alpha * p(component);
                const Float r_new = r_read(component) - alpha * Ap(component);
                r_out(component)  = r_new;
                r[component]      = static_cast<float>(r_new);
            }
        }
        else
        {
            r[0] = static_cast<float>(r_read(0));
            r[1] = static_cast<float>(r_read(1));
            r[2] = static_cast<float>(r_read(2));
        }
    }

    int lane_id       = threadIdx.x % BANKSIZE;
    int local_warp_id = threadIdx.x / BANKSIZE;
    int global_warp   = pdx / BANKSIZE;

    if(in_range)
        multi_lr(pdx) = r;

    __shared__ float sum_residual_s[DEFAULT_BLOCKSIZE * 3];
    __shared__ int   prefix_sum_raw[DEFAULT_WARPNUM];
    auto             sum_residual = make_dense_1d(sum_residual_s);
    auto             prefix_sum_v = make_dense_1d(prefix_sum_raw);

    if(lane_id == 0)
        prefix_sum_v(local_warp_id) = in_range ? prefix_orig(global_warp) : 0;

    const unsigned int group_mask                 = mas_logical_group_mask();
    sum_residual(threadIdx.x)                     = 0;
    sum_residual(threadIdx.x + DEFAULT_BLOCKSIZE) = 0;
    sum_residual(threadIdx.x + 2 * DEFAULT_BLOCKSIZE) = 0;
    __syncwarp(group_mask);

    unsigned int connect_msk = is_valid ? fine_conn(idx) : 0U;

    if(prefix_sum_v(local_warp_id) == 1)
    {
        using WarpReduceF = cub::WarpReduce<float, BANKSIZE>;
        __shared__ typename WarpReduceF::TempStorage temp_reduce_f[DEFAULT_WARPNUM];

        r[0] = WarpReduceF(temp_reduce_f[local_warp_id]).Sum(r[0]);
        __syncwarp(group_mask);
        r[1] = WarpReduceF(temp_reduce_f[local_warp_id]).Sum(r[1]);
        __syncwarp(group_mask);
        r[2] = WarpReduceF(temp_reduce_f[local_warp_id]).Sum(r[2]);

        if(lane_id == 0 && is_valid)
        {
            int cur = idx;
            for(int l = 0; l < level_num - 1; l++)
            {
                cur = going_next(cur);

                UIPC_KERNEL_ASSERT(cur >= level_size(l + 1).y
                                       && cur < level_size(l + 2).y,
                                   "build_R: cur=%d not in level %d [%d, %d)",
                                   cur,
                                   l + 1,
                                   level_size(l + 1).y,
                                   level_size(l + 2).y);

                atomicAdd(&(multi_lr(cur)[0]), r[0]);
                atomicAdd(&(multi_lr(cur)[1]), r[1]);
                atomicAdd(&(multi_lr(cur)[2]), r[2]);
            }
        }
    }
    else
    {
        int elected_lane = is_valid ? __ffs(connect_msk) - 1 : 0;

        if(is_valid)
        {
            atomicAdd(&sum_residual(local_warp_id * BANKSIZE + elected_lane), r[0]);
            atomicAdd(&sum_residual(local_warp_id * BANKSIZE + elected_lane + DEFAULT_BLOCKSIZE),
                      r[1]);
            atomicAdd(&sum_residual(local_warp_id * BANKSIZE + elected_lane + 2 * DEFAULT_BLOCKSIZE),
                      r[2]);
        }

        __syncwarp(group_mask);

        unsigned int elected_prefix = __popc(connect_msk & lanemask_lt(lane_id));
        if(is_valid && elected_prefix == 0)
        {
            int cur = idx;
            for(int l = 0; l < level_num - 1; l++)
            {
                cur = going_next(cur);

                UIPC_KERNEL_ASSERT(cur >= level_size(l + 1).y
                                       && cur < level_size(l + 2).y,
                                   "build_R: cur=%d not in level %d [%d, %d)",
                                   cur,
                                   l + 1,
                                   level_size(l + 1).y,
                                   level_size(l + 2).y);

                atomicAdd(&(multi_lr(cur)[0]), sum_residual(threadIdx.x));
                atomicAdd(&(multi_lr(cur)[1]), sum_residual(threadIdx.x + DEFAULT_BLOCKSIZE));
                atomicAdd(&(multi_lr(cur)[2]),
                          sum_residual(threadIdx.x + DEFAULT_BLOCKSIZE * 2));
            }
        }
    }
}


// Prolongation and r^T z of the fused PCG step. Partitioned vertices hold
// the r updated by build_multi_level_R<true>. The MAS passes never touch
// an unpartitioned vertex, so with a diagonal fallback (non-empty
// unpart_diag_inv) its x/r update and z = D^{-1} r happen here.
__global__ void former_collect_final_Z_dot_kernel(
    cuda_tool::DenseVectorView<Float>  X_view,
    cuda_tool::CDenseVectorView<Float> P_view,
    cuda_tool::DenseVectorView<Float>  Z_view,
    cuda_tool::DenseVectorView<Float>  R_view,
    cuda_tool::CDenseVectorView<Float> Ap_view,
    cuda_tool::CDense<Float>           rz,
    cuda_tool::CDense<Float>           pAp,
    cuda_tool::CBufferView<float3>     multi_lz,
    cuda_tool::CBufferView<LevelTable> coarse_table,
    cuda_tool::CBufferView<int>        real_to_part,
    cuda_tool::CBufferView<Matrix3x3>  unpart_diag_inv,
    cuda_tool::Dense<Float>            rz_new,
    cuda_tool::CDense<IndexT>          converged,
    int                                level_num,
    int                                n)
{
    using BlockReduce = cub::BlockReduce<Float, DEFAULT_BLOCKSIZE>;
    __shared__ typename BlockReduce::TempStorage reduce_storage;

    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    Float     dot = Float{0};

    if(idx < n && *converged == 0)
    {
        const int rdx = real_to_part(idx);
        if(rdx < 0 && unpart_diag_inv.size() != 0)
        {
            const Float alpha = *rz / *pAp;
            auto        x     = X_view.segment<3>(3 * idx);
            auto        p     = P_view.segment<3>(3 * idx);
            auto        r     = R_view.segment<3>(3 * idx);
            auto        Ap    = Ap_view.segment<3>(3 * idx);
            Vector3     r_new;
#pragma unroll
            for(int component = 0; component < 3; ++component)
            {
                x(component) += alpha * p(component);
                r_new(component) = r(component) - alpha * Ap(component);
                r(component)     = r_new(component);
            }

            const Vector3 z_new = unpart_diag_inv(idx) * r_new;
            auto          z     = Z_view.segment<3>(3 * idx);
            z(0)                = z_new(0);
            z(1)                = z_new(1);
            z(2)                = z_new(2);
            dot = r_new(0) * z_new(0) + r_new(1) * z_new(1) + r_new(2) * z_new(2);
        }
        else if(rdx >= 0)
        {
            float3           cz    = multi_lz(rdx);
            const LevelTable table = coarse_table(idx);
            for(int l = 1; l < level_num; ++l)
            {
                const float3 val = multi_lz(table.index[l - 1]);
                cz.x += val.x;
                cz.y += val.y;
                cz.z += val.z;
            }

            auto z = Z_view.segment<3>(3 * idx);
            auto r = R_view.segment<3>(3 * idx);
            z(0)   = static_cast<Float>(cz.x);
            z(1)   = static_cast<Float>(cz.y);
            z(2)   = static_cast<Float>(cz.z);
            dot    = r(0) * z(0) + r(1) * z(1) + r(2) * z(2);
        }
    }

    const Float block_sum = BlockReduce(reduce_storage).Sum(dot);
    if(threadIdx.x == 0 && block_sum != Float{0})
        atomicAdd(rz_new.data(), block_sum);
}

}  // namespace

TEST_CASE("MAS fused PCG step keeps the bits of the former restriction and prolongation on exact data",
          "[build_solve_focused][mas]")
{
    using Access = MASPreconditionerEngineTestAccess;
    using SymF   = MASPreconditionerEngine::ClusterMatrixSymF;

    // Exactly representable data: x, p multiples of 1/16; r and Ap
    // multiples of 1/16 below 1/2 (on the 5000-vertex chain only every 61st
    // vertex nonzero, so a coarse residual sums at most 82 of them);
    // inverse entries in {-1, -1/2, 0, 1/2, 1}; alpha = 1/4. Every float and
    // double sum is then exact, so the coarse-level atomics, whose arrival
    // order differs run to run (and with the block shape), cannot change a
    // bit, and old and new kernels must agree everywhere.
    for(int setup = 0; setup < 2; ++setup)
    {
        CAPTURE(setup);
        MASPreconditionerEngine            engine;
        UnpartitionedGapChain              gap;
        cuda_tool::DeviceBuffer<Matrix3x3> d_diag;  // gap chain: the fallback
        if(setup == 0)
        {
            constexpr int             Vertices = 5000;
            std::vector<unsigned int> starts(Vertices), counts(Vertices), list;
            std::vector<int>          identity(Vertices);
            auto                      linked = [](int v)
            { return (static_cast<unsigned int>(v) * 2654435761u) % 9u != 0u; };
            for(int v = 0; v < Vertices; ++v)
            {
                starts[v] = static_cast<unsigned int>(list.size());
                if(v > 0 && linked(v - 1))
                    list.push_back(v - 1);
                if(v + 1 < Vertices && linked(v))
                    list.push_back(v + 1);
                counts[v] = static_cast<unsigned int>(list.size()) - starts[v];
            }
            std::iota(identity.begin(), identity.end(), 0);
            engine.init_neighbor(Vertices,
                                 static_cast<int>(list.size()),
                                 Vertices,
                                 span<const unsigned int>{list},
                                 span<const unsigned int>{starts},
                                 span<const unsigned int>{counts},
                                 span<const int>{identity},
                                 span<const int>{identity});
            engine.init_matrix();
            cuda_tool::DeviceBuffer<Matrix3x3> d_values(1);
            cuda_tool::DeviceBuffer<int>       d_rows(1), d_cols(1);
            const Matrix3x3                    one  = Matrix3x3::Identity();
            const int                          zero = 0;
            d_values.view().copy_from(&one);
            d_rows.view().copy_from(&zero);
            d_cols.view().copy_from(&zero);
            engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);
        }
        else
        {
            gap.init_engine(engine);
            gap.assemble(engine);
        }
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

        const int    nodes   = Access::total_nodes(engine);
        const int    scalars = 3 * nodes;
        std::mt19937 rng(31 + setup);
        auto         dyadic = [&](int span)
        { return double(int(rng() % (2 * span + 1)) - span) / 16; };

        auto&             inv = Access::inverses(engine);
        std::vector<SymF> h_inv(inv.size());
        for(auto& m : h_inv)
            for(auto& b : m.M)
                for(int k = 0; k < 9; ++k)
                    b.data()[k] = float(int(rng() % 5) - 2) / 2;
        inv.view().copy_from(h_inv.data());

        std::vector<Float> h_x(scalars), h_p(scalars), h_r(scalars), h_ap(scalars);
        for(int i = 0; i < scalars; ++i)
        {
            h_x[i]              = dyadic(64);
            h_p[i]              = dyadic(64);
            const bool residual = setup == 1 || (i / 3) % 61 == 0;
            h_r[i]              = residual ? dyadic(8) : 0.0;
            h_ap[i]             = residual ? dyadic(8) : 0.0;
        }
        std::vector<Matrix3x3> h_diag;
        if(setup == 1)
        {
            h_diag.resize(nodes);
            for(auto& d : h_diag)
                for(int k = 0; k < 9; ++k)
                    d.data()[k] = dyadic(16);
            d_diag.resize(nodes);
            d_diag.view().copy_from(h_diag.data());
        }

        // two copies of every vector the step writes
        std::array<cuda_tool::DeviceDenseVector<Float>, 2> d_x, d_r, d_z;
        cuda_tool::DeviceDenseVector<Float>                d_p, d_ap;
        for(auto* v : {&d_p, &d_ap})
            v->resize(scalars);
        d_p.buffer_view().copy_from(h_p.data());
        d_ap.buffer_view().copy_from(h_ap.data());
        for(int k = 0; k < 2; ++k)
        {
            for(auto* v : {&d_x[k], &d_r[k], &d_z[k]})
                v->resize(scalars);
            d_x[k].buffer_view().copy_from(h_x.data());
            d_r[k].buffer_view().copy_from(h_r.data());
            REQUIRE(cudaMemset(d_z[k].buffer_view().data(), 0, scalars * sizeof(Float))
                    == cudaSuccess);
        }
        cuda_tool::DeviceVar<Float> d_rz{Float{1}}, d_pap{Float{4}};
        std::array<cuda_tool::DeviceVar<Float>, 2> d_rz_new{
            cuda_tool::DeviceVar<Float>{Float{0}}, cuda_tool::DeviceVar<Float>{Float{0}}};
        cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

        // reference state: residual and solution levels copied before the
        // engine's step runs (its coarse levels are zero after the first use)
        auto& R = Access::residuals(engine);
        auto& Z = Access::solutions(engine);
        REQUIRE(cudaMemset(R.data(), 0, R.size() * sizeof(Eigen::Vector3f)) == cudaSuccess);
        REQUIRE(cudaMemset(Z.data(), 0, Z.size() * sizeof(float3)) == cudaSuccess);
        cuda_tool::DeviceBuffer<Eigen::Vector3f> ref_R(R.size());
        cuda_tool::DeviceBuffer<float3>          ref_Z(Z.size());
        REQUIRE(cudaMemset(ref_R.data(), 0, ref_R.size() * sizeof(Eigen::Vector3f)) == cudaSuccess);
        REQUIRE(cudaMemset(ref_Z.data(), 0, ref_Z.size() * sizeof(float3)) == cudaSuccess);

        const int map_nodes = Access::total_map_nodes(engine);
        const int levels    = Access::level_num(engine);
        const int clusters  = Access::total_clusters(engine);
        former_build_multi_level_R_kernel<true>
            <<<(map_nodes + DEFAULT_BLOCKSIZE - 1) / DEFAULT_BLOCKSIZE, DEFAULT_BLOCKSIZE>>>(
                d_x[0].view(),
                d_p.cview(),
                d_r[0].view(),
                d_r[0].cview(),
                d_ap.cview(),
                d_rz.cviewer(),
                d_pap.cviewer(),
                ref_R.view(),
                Access::going_next_buffer(engine).cview(0, clusters),
                Access::level_sizes(engine).cview(),
                Access::prefix_original(engine).cview(),
                Access::fine_connect_masks(engine).cview(),
                Access::part_to_real(engine).cview(),
                d_converged.cviewer(),
                levels,
                map_nodes);
        const int skip_count = Access::skip_count(engine);
        reference_schwarz_local_solve_kernel<<<clusters / SchwarzBank - skip_count, DEFAULT_BLOCKSIZE>>>(
            inv.cview(),
            ref_R.view(),
            ref_Z.view(),
            d_converged.cviewer(),
            Access::level1_begin(engine),
            Access::skip_begin(engine),
            skip_count,
            clusters * SchwarzBank);
        former_collect_final_Z_dot_kernel<<<(nodes + DEFAULT_BLOCKSIZE - 1) / DEFAULT_BLOCKSIZE, DEFAULT_BLOCKSIZE>>>(
            d_x[0].view(),
            d_p.cview(),
            d_z[0].view(),
            d_r[0].view(),
            d_ap.cview(),
            d_rz.cviewer(),
            d_pap.cviewer(),
            ref_Z.cview(),
            Access::coarse_tables(engine).cview(),
            Access::real_to_part(engine).cview(),
            d_diag.cview(),
            d_rz_new[0].viewer(),
            d_converged.cviewer(),
            levels,
            nodes);

        engine.fused_pcg_update_apply_dot(d_x[1].view(),
                                          d_p.cview(),
                                          d_r[1].view(),
                                          d_ap.cview(),
                                          d_z[1].view(),
                                          d_rz.cview(),
                                          d_pap.cview(),
                                          d_rz_new[1].view(),
                                          d_converged.cview(),
                                          nullptr,
                                          d_diag.cview());
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

        auto host = [&](const cuda_tool::DeviceDenseVector<Float>& v)
        {
            std::vector<Float> h(scalars);
            v.buffer_view().copy_to(h.data());
            return h;
        };
        for(auto* pair : {&d_x, &d_r, &d_z})
        {
            const auto a = host((*pair)[0]), b = host((*pair)[1]);
            CHECK(std::memcmp(a.data(), b.data(), scalars * sizeof(Float)) == 0);
        }
        const Float rz_ref = d_rz_new[0], rz_now = d_rz_new[1];
        CHECK(std::memcmp(&rz_ref, &rz_now, sizeof(Float)) == 0);
        CHECK(rz_now != Float{0});
        std::vector<Eigen::Vector3f> r_ref(R.size()), r_now(R.size());
        std::vector<float3>          z_ref(Z.size()), z_now(Z.size());
        ref_R.view().copy_to(r_ref.data());
        R.view().copy_to(r_now.data());
        ref_Z.view().copy_to(z_ref.data());
        Z.view().copy_to(z_now.data());
        CHECK(std::memcmp(r_ref.data(), r_now.data(), r_now.size() * sizeof(Eigen::Vector3f))
              == 0);
        CHECK(std::memcmp(z_ref.data(), z_now.data(), z_now.size() * sizeof(float3)) == 0);
    }
}
