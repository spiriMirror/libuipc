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
#include <linear_system/spmv.h>

#include <array>
#include <cstring>
#include <limits>
#include <random>
#include <utility>
#include <vector>
#include <cub/warp/warp_reduce.cuh>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

TEST_CASE("fused PCG pipelined SpMV preserves result and clears the next slot",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    constexpr int BlockRows = 2;
    constexpr int Dofs      = BlockRows * 3;

    const std::array<int, 3> rows = {0, 0, 1};
    const std::array<int, 3> cols = {0, 1, 1};
    std::array<Matrix3x3, 3> values;
    values[0] = Float{2} * Matrix3x3::Identity();
    values[1] = Matrix3x3::Identity();
    values[2] = Float{3} * Matrix3x3::Identity();

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(BlockRows, BlockRows, rows.size());
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    const std::array<Float, Dofs>       x_host = {1, 2, 3, 4, 5, 6};
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());
    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    dot           = Float{0};
    next_dot      = Float{11};
    converged     = IndexT{0};
    triplet_count = static_cast<IndexT>(rows.size());

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    const std::array<Float, Dofs> expected_y = {6, 9, 12, 13, 17, 21};
    std::array<Float, Dofs>       actual_y{};
    std::array<Float, Dofs>       actual_next_y{};
    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Catch::Approx(323.0).margin(1e-12));
    REQUIRE(static_cast<Float>(next_dot) == Float{0});

    // Once converged, neither the output nor the next-slot clear is allowed
    // to run. This is the skip used by later inactive iterations in a replay.
    y.buffer_view().fill(Float{13});
    next_y.buffer_view().fill(Float{17});
    dot       = Float{19};
    next_dot  = Float{23};
    converged = IndexT{1};

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Float{13});
        REQUIRE(actual_next_y[i] == Float{17});
    }
    REQUIRE(static_cast<Float>(dot) == Float{19});
    REQUIRE(static_cast<Float>(next_dot) == Float{23});
}

TEST_CASE("fused PCG pipelined SpMV accumulates across multiple CTA32 blocks",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    constexpr int TripletCount = 65;
    constexpr int Dofs         = 3;

    std::array<int, TripletCount>       rows{};
    std::array<int, TripletCount>       cols{};
    std::array<Matrix3x3, TripletCount> values;
    for(auto& value : values)
        value = Matrix3x3::Identity();

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(1, 1, TripletCount);
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    const std::array<Float, Dofs>       x_host = {1, 2, 3};
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());
    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    dot           = Float{0};
    next_dot      = Float{11};
    converged     = IndexT{0};
    triplet_count = TripletCount;

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    const std::array<Float, Dofs> expected_y = {65, 130, 195};
    std::array<Float, Dofs>       actual_y{};
    std::array<Float, Dofs>       actual_next_y{};
    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Catch::Approx(910.0).margin(1e-12));
    REQUIRE(static_cast<Float>(next_dot) == Float{0});
}

TEST_CASE("fused PCG pipelined SpMV honors active count and clears a zero-count ping-pong slot",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    constexpr int BlockRows     = 2;
    constexpr int Dofs          = BlockRows * 3;
    constexpr int ActiveCount   = 3;
    constexpr int ReservedCount = 67;

    std::array<int, ReservedCount>       rows{};
    std::array<int, ReservedCount>       cols{};
    std::array<Matrix3x3, ReservedCount> values;
    for(auto& value : values)
        value = Float{100} * Matrix3x3::Identity();

    rows[0]   = 0;
    cols[0]   = 0;
    values[0] = Float{2} * Matrix3x3::Identity();
    rows[1]   = 0;
    cols[1]   = 1;
    values[1] = Matrix3x3::Identity();
    rows[2]   = 1;
    cols[2]   = 1;
    values[2] = Float{3} * Matrix3x3::Identity();

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(BlockRows, BlockRows, ReservedCount);
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    const std::array<Float, Dofs>       x_host = {1, 2, 3, 4, 5, 6};
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());
    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    dot           = Float{0};
    next_dot      = Float{11};
    converged     = IndexT{0};
    triplet_count = ActiveCount;

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    const std::array<Float, Dofs> expected_y = {6, 9, 12, 13, 17, 21};
    std::array<Float, Dofs>       actual_y{};
    std::array<Float, Dofs>       actual_next_y{};
    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Catch::Approx(323.0).margin(1e-12));
    REQUIRE(static_cast<Float>(next_dot) == Float{0});

    // Mirror the Graph ping-pong: the just-cleared slot becomes the current
    // output, while the previous nonzero y/pAp slot must be cleared for reuse.
    triplet_count = IndexT{0};
    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      next_y.view(),
                                      next_dot.view(),
                                      y.view(),
                                      dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Float{0});
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Float{0});
    REQUIRE(static_cast<Float>(next_dot) == Float{0});
}

TEST_CASE("fused PCG SpMV skips warps past the live count without changing results",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    // 37 live triplets straddle the first two CTA32 warps; the remaining
    // capacity holds in-range indices with NaN values, so any warp that
    // processed them would poison y or the dot. next_y is longer than the
    // whole grid to exercise the grid-stride clear before the early exit.
    constexpr int BlockRows     = 120;
    constexpr int Dofs          = BlockRows * 3;
    constexpr int LiveCount     = 37;
    constexpr int ReservedCount = 160;

    std::vector<int>       rows(ReservedCount, 0);
    std::vector<int>       cols(ReservedCount, 1);
    std::vector<Matrix3x3> values(
        ReservedCount, Matrix3x3::Constant(std::numeric_limits<Float>::quiet_NaN()));
    int live = 0;
    for(int i = 0; live < LiveCount; ++i)
    {
        for(int j : {i, i + 1, i + 4})
        {
            if(live == LiveCount || j >= BlockRows)
                continue;
            rows[live] = i;
            cols[live] = j;
            Matrix3x3 value;
            for(int r = 0; r < 3; ++r)
                for(int c = 0; c < 3; ++c)
                    value(r, c) = Float(1 + ((7 * live + 3 * r + c) % 11)) / 8;
            if(i == j)
                value = (value + value.transpose()).eval();
            values[live++] = value;
        }
    }

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(BlockRows, BlockRows, ReservedCount);
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    std::vector<Float> x_host(Dofs);
    for(int i = 0; i < Dofs; ++i)
        x_host[i] = Float(1 + (5 * i) % 13) / 4;

    std::vector<Float> expected_y(Dofs, Float{0});
    Float              expected_dot = 0;
    for(int t = 0; t < LiveCount; ++t)
    {
        const int     i   = rows[t];
        const int     j   = cols[t];
        const Vector3 x_i = Eigen::Map<const Vector3>(x_host.data() + 3 * i);
        const Vector3 x_j = Eigen::Map<const Vector3>(x_host.data() + 3 * j);
        const Vector3 a   = values[t] * x_j;
        Eigen::Map<Vector3>(expected_y.data() + 3 * i) += a;
        if(i != j)
            Eigen::Map<Vector3>(expected_y.data() + 3 * j) += values[t].transpose() * x_i;
        expected_dot += (i == j ? Float{1} : Float{2}) * x_i.dot(a);
    }

    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    converged     = IndexT{0};
    triplet_count = LiveCount;
    REQUIRE(A.triplet_capacity() < static_cast<SizeT>(Dofs));

    auto check = [&](const char* path)
    {
        INFO(path);
        std::vector<Float> actual_y(Dofs);
        y.buffer_view().copy_to(actual_y.data());
        for(int i = 0; i < Dofs; ++i)
            REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(static_cast<Float>(dot) == Catch::Approx(expected_dot).epsilon(1e-12));
    };

    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});
    dot      = Float{0};
    next_dot = Float{11};
    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);
    check("pipelined");
    std::vector<Float> actual_next_y(Dofs);
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
        REQUIRE(actual_next_y[i] == Float{0});
    REQUIRE(static_cast<Float>(next_dot) == Float{0});

    y.buffer_view().fill(Float{5});
    dot = Float{3};
    Spmv().rbk_sym_spmv_dot(Float{1},
                            A.cview(),
                            x.cview(),
                            Float{0},
                            y.view(),
                            dot.view(),
                            triplet_count.cviewer(),
                            A.triplet_capacity(),
                            nullptr);
    check("plain");
}

TEST_CASE("fused PCG SpMV with a unit scale computes the general kernel's bits",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    // a == 1 takes a kernel without the scale multiplications. The general
    // kernel is checked through a == 1/2: halving every scaled term is exact,
    // so it must return exactly half of the unit kernel's y and dot. The
    // matrix fits in one warp and every y block receives exactly one atomic
    // (row heads 0..15 and the otherwise unused columns 40..61), so both
    // launches are deterministic and comparable bit for bit.
    constexpr int BlockRows = 64;
    constexpr int Dofs      = BlockRows * 3;

    std::vector<int>       rows;
    std::vector<int>       cols;
    std::vector<Matrix3x3> values;
    // (has a diagonal block, off-diagonal blocks) per row, cycled
    const std::array<std::pair<bool, int>, 5> pattern  = {std::pair{true, 2},
                                                          std::pair{false, 3},
                                                          std::pair{true, 0},
                                                          std::pair{false, 1},
                                                          std::pair{true, 1}};
    int                                       next_col = 40;
    for(int row = 0; rows.size() < 32; ++row)
    {
        const auto [diag, off] = pattern[row % pattern.size()];
        for(int k = -1; k < off && rows.size() < 32; ++k)
        {
            if(k < 0 && !diag)
                continue;
            const int col = k < 0 ? row : next_col++;
            Matrix3x3 value;
            for(int r = 0; r < 3; ++r)
                for(int c = 0; c < 3; ++c)
                    value(r, c) =
                        Float(1 + ((7 * (int)rows.size() + 3 * r + c) % 11)) / 9;
            rows.push_back(row);
            cols.push_back(col);
            values.push_back(value);
        }
    }
    REQUIRE(rows.size() == 32);
    REQUIRE(next_col <= BlockRows);

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(BlockRows, BlockRows, rows.size());
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    std::vector<Float> x_host(Dofs);
    for(int i = 0; i < Dofs; ++i)
        x_host[i] = Float(1 + (5 * i) % 13) / 7;

    std::vector<Float> expected_y(Dofs, Float{0});
    Float              expected_dot = 0;
    for(SizeT t = 0; t < rows.size(); ++t)
    {
        const int     i   = rows[t];
        const int     j   = cols[t];
        const Vector3 x_i = Eigen::Map<const Vector3>(x_host.data() + 3 * i);
        const Vector3 x_j = Eigen::Map<const Vector3>(x_host.data() + 3 * j);
        const Vector3 a   = values[t] * x_j;
        Eigen::Map<Vector3>(expected_y.data() + 3 * i) += a;
        if(i != j)
            Eigen::Map<Vector3>(expected_y.data() + 3 * j) += values[t].transpose() * x_i;
        expected_dot += (i == j ? Float{1} : Float{2}) * x_i.dot(a);
    }

    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    converged     = IndexT{0};
    triplet_count = static_cast<IndexT>(rows.size());

    auto run = [&](Float scale, bool pipelined)
    {
        y.buffer_view().fill(Float{0});
        next_y.buffer_view().fill(Float{7});
        dot      = Float{0};
        next_dot = Float{11};
        if(pipelined)
            Spmv().rbk_sym_spmv_dot_pipelined(scale,
                                              A.cview(),
                                              x.cview(),
                                              y.view(),
                                              dot.view(),
                                              next_y.view(),
                                              next_dot.view(),
                                              converged.view(),
                                              triplet_count.cviewer(),
                                              A.triplet_capacity(),
                                              nullptr);
        else
            Spmv().rbk_sym_spmv_dot(scale,
                                    A.cview(),
                                    x.cview(),
                                    Float{0},
                                    y.view(),
                                    dot.view(),
                                    triplet_count.cviewer(),
                                    A.triplet_capacity(),
                                    nullptr);
        std::vector<Float> result(Dofs + 1);
        y.buffer_view().copy_to(result.data());
        result[Dofs] = static_cast<Float>(dot);
        return result;
    };

    for(bool pipelined : {true, false})
    {
        INFO("pipelined " << pipelined);
        const auto unit = run(Float{1}, pipelined);
        const auto half = run(Float{0.5}, pipelined);
        for(int i = 0; i < Dofs; ++i)
        {
            REQUIRE(unit[i] == Catch::Approx(expected_y[i]).margin(1e-12));
            REQUIRE(half[i] == Float{0.5} * unit[i]);
        }
        REQUIRE(unit[Dofs] == Catch::Approx(expected_dot).epsilon(1e-12));
        REQUIRE(half[Dofs] == Float{0.5} * unit[Dofs]);
    }
}

namespace
{
struct ReferenceSpmvFlags
{
    union
    {
        struct
        {
            unsigned char is_head;
            unsigned char is_cross_warp;
            unsigned char is_valid;
        };
        unsigned int flags;
    };
};

__host__ __device__ constexpr int reference_b2i(bool b)
{
    return b ? 1 : 0;
}

// The fused SpMV/dot kernel before it shared one dot path between diagonal
// and off-diagonal blocks and stopped its segmented sums at the warp's
// longest segment (CUB HeadSegmentedReduce, all five steps), kept as the
// bitwise reference. Block size 32, as the library launches it.
template <bool Pipelined, bool UnitScale>
__global__ void reference_spmv_dot_kernel(Float a,
                                          cuda_tool::CBCOOMatrixView<Float, 3> A,
                                          cuda_tool::CDenseVectorView<Float> x,
                                          cuda_tool::DenseVectorView<Float>  y,
                                          cuda_tool::Dense<Float> d_dot,
                                          cuda_tool::CDense<IndexT> d_triplet_count,
                                          Float*        next_y,
                                          int           next_y_size,
                                          Float*        next_dot,
                                          const IndexT* converged)
{
    auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
    auto thread_id_in_block = threadIdx.x;

    if constexpr(Pipelined)
    {
        // Every thread observes the same stream-ordered scalar. Returning
        // here is therefore uniform across the block and cannot strand a
        // later collective.
        if(*converged != 0)
            return;

        for(int i = global_thread_id; i < next_y_size; i += gridDim.x * blockDim.x)
            next_y[i] = Float{0};
        if(global_thread_id == 0)
            *next_dot = Float{0};
    }

    // count lives on device: a graph capturing this kernel then stays
    // valid when the matrix nnz changes within the reserved capacity
    const int     triplet_count = (int)(*d_triplet_count);
    constexpr int warp_size     = 32;

    // The grid covers the capacity, which can be several times the live
    // count. A warp wholly past the count writes nothing, so it leaves
    // here (warp-uniform) instead of running the FP64 warp reductions.
    if((int)(global_thread_id & ~(warp_size - 1u)) >= triplet_count)
        return;
    constexpr int block_dim = 32;
    constexpr int N         = 3;
    using T                 = Float;

    using WarpReduceInt   = cub::WarpReduce<int, warp_size>;
    using WarpReduceFloat = cub::WarpReduce<Float, warp_size>;

    auto warp_id = thread_id_in_block / warp_size;
    auto lane_id = thread_id_in_block & (warp_size - 1);

    __shared__ union
    {
        typename WarpReduceInt::TempStorage temp_storage_int[block_dim / warp_size];
        typename WarpReduceFloat::TempStorage temp_storage_float[block_dim / warp_size];
    };

    int                prev_i = -1;
    int                i      = -1;
    ReferenceSpmvFlags flags;
    Vector3            vec;
    vec.setZero();
    Float dot_local = 0;

    flags.is_cross_warp = 0;
    flags.is_valid      = 0;
    flags.is_head       = 0;

    // raw triplet arrays: the view's A(i) accessor bounds-checks against
    // the count baked at capture time, while the device-side count may be
    // larger (matrix grew within capacity) — read through the pointers
    const int*       rows = A.row_indices().data();
    const int*       cols = A.col_indices().data();
    const Matrix3x3* vals = A.values().data();
    if(global_thread_id < triplet_count)
    {
        if(global_thread_id > 0)
            prev_i = rows[global_thread_id - 1];

        i      = rows[global_thread_id];
        auto j = cols[global_thread_id];

        Eigen::Vector<T, N> x_j = x.segment<N>(j * N).as_eigen();
        vec                     = vals[global_thread_id] * x_j;

        flags.is_valid = 1;

        if(i == j)
        {
            if constexpr(UnitScale)
                dot_local = x_j.dot(vec);
            else
                dot_local = a * x_j.dot(vec);
        }
        else
        {
            Eigen::Vector<T, N> x_i = x.segment<N>(i * N).as_eigen();
            Vector3             vec_;
            if constexpr(UnitScale)
            {
                dot_local = 2.0 * x_i.dot(vec);
                vec_      = vals[global_thread_id].transpose() * x_i;
            }
            else
            {
                dot_local = 2.0 * a * x_i.dot(vec);
                vec_      = a * vals[global_thread_id].transpose() * x_i;
            }
            y.segment<N>(j * N).atomic_add(vec_);
        }

        if(lane_id == 0)
            flags.is_head = 1;
        else
            flags.is_head = reference_b2i(prev_i != i);
    }

    vec.x() = WarpReduceFloat(temp_storage_float[warp_id])
                  .HeadSegmentedReduce(vec.x(),
                                       flags.is_head,
                                       [](Float a, Float b) { return a + b; });

    vec.y() = WarpReduceFloat(temp_storage_float[warp_id])
                  .HeadSegmentedReduce(vec.y(),
                                       flags.is_head,
                                       [](Float a, Float b) { return a + b; });

    vec.z() = WarpReduceFloat(temp_storage_float[warp_id])
                  .HeadSegmentedReduce(vec.z(),
                                       flags.is_head,
                                       [](Float a, Float b) { return a + b; });

    if(flags.is_head && flags.is_valid)
    {
        auto seg_y = y.segment<N>(i * N);
        if constexpr(UnitScale)
            seg_y.atomic_add(vec);
        else
            seg_y.atomic_add((a * vec).eval());
    }

    dot_local = WarpReduceFloat(temp_storage_float[warp_id]).Sum(dot_local);

    if(lane_id == 0 && dot_local != Float{0})
        atomicAdd(d_dot.data(), dot_local);
}

}  // namespace

TEST_CASE("fused PCG SpMV keeps the bits of the branch-per-block kernel with full segmented sums",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    // Warps of 32 blocks whose rows never span two warps, with segments of
    // 1..32 blocks, rows with and without a diagonal block, and off-diagonal
    // columns used by no other block: every y block receives exactly one
    // atomic, so y is deterministic and comparable bit for bit. pAp gets one
    // atomic per warp: bitwise for single-warp launches, to rounding across
    // warps.
    std::mt19937_64                       rng(20261028);
    std::uniform_real_distribution<Float> u(-1.0, 1.0);

    auto make_matrix = [&](int                     warps,
                           int                     tail,
                           std::vector<int>&       rows,
                           std::vector<int>&       cols,
                           std::vector<Matrix3x3>& values,
                           int&                    block_rows)
    {
        rows.clear();
        cols.clear();
        values.clear();
        int next_row = 0;
        std::vector<std::pair<int, bool>> row_heads;  // (row, starts with its diagonal)
        for(int w = 0; w < warps; ++w)
        {
            const int    count = w + 1 == warps && tail > 0 ? tail : 32;
            const double p     = std::array{0.02, 0.1, 0.3, 0.7, 1.0}[w % 5];
            std::bernoulli_distribution new_segment(p), diagonal(0.5);
            for(int lane = 0; lane < count; ++lane)
            {
                if(lane == 0 || new_segment(rng))
                    row_heads.emplace_back(next_row++, diagonal(rng));
                rows.push_back(row_heads.back().first);
            }
        }
        // columns: the row itself for a leading diagonal block, else a column
        // of its own past every row
        int next_col = next_row;
        for(SizeT t = 0; t < rows.size(); ++t)
        {
            const bool head = t == 0 || rows[t - 1] != rows[t];
            const bool diag = head && row_heads[rows[t]].second;
            cols.push_back(diag ? rows[t] : next_col++);
            Matrix3x3 m;
            for(int r = 0; r < 3; ++r)
                for(int c = 0; c < 3; ++c)
                    m(r, c) = u(rng) * std::pow(Float{10}, int(t % 9) - 4);
            values.push_back(m);
        }
        block_rows = next_col;
    };

    auto run = [&](const std::vector<int>&       rows,
                   const std::vector<int>&       cols,
                   const std::vector<Matrix3x3>& values,
                   int                           block_rows,
                   Float                         scale,
                   bool                          pipelined,
                   bool                          reference,
                   std::vector<Float>&           y_out,
                   Float&                        dot_out)
    {
        const int                             n = (int)rows.size();
        cuda_tool::DeviceBCOOMatrix<Float, 3> A;
        A.resize(block_rows, block_rows, n);
        A.row_indices().copy_from(rows.data());
        A.col_indices().copy_from(cols.data());
        A.values().copy_from(values.data());
        std::vector<Float> x_host(3 * block_rows);
        std::mt19937_64    xr(7);
        for(auto& v : x_host)
            v = u(xr) * std::pow(Float{10}, int(xr() % 7) - 3);
        cuda_tool::DeviceDenseVector<Float> x, y, next_y;
        x.resize(3 * block_rows);
        y.resize(3 * block_rows);
        next_y.resize(3 * block_rows);
        x.buffer_view().copy_from(x_host.data());
        y.buffer_view().fill(Float{0});
        next_y.buffer_view().fill(Float{5});
        cuda_tool::DeviceVar<Float> dot{Float{0}}, next_dot{Float{3}};
        cuda_tool::DeviceVar<IndexT> converged{IndexT{0}}, triplet_count{IndexT(n)};
        if(reference)
        {
            const int blocks = (n + 31) / 32;
            if(pipelined)
            {
                if(scale == Float{1})
                    reference_spmv_dot_kernel<true, true>
                        <<<blocks, 32>>>(scale,
                                         A.cview(),
                                         x.cview(),
                                         y.view(),
                                         dot.viewer(),
                                         triplet_count.cviewer(),
                                         next_y.buffer_view().data(),
                                         3 * block_rows,
                                         next_dot.data(),
                                         converged.data());
                else
                    reference_spmv_dot_kernel<true, false>
                        <<<blocks, 32>>>(scale,
                                         A.cview(),
                                         x.cview(),
                                         y.view(),
                                         dot.viewer(),
                                         triplet_count.cviewer(),
                                         next_y.buffer_view().data(),
                                         3 * block_rows,
                                         next_dot.data(),
                                         converged.data());
            }
            else if(scale == Float{1})
                reference_spmv_dot_kernel<false, true>
                    <<<blocks, 32>>>(scale,
                                     A.cview(),
                                     x.cview(),
                                     y.view(),
                                     dot.viewer(),
                                     triplet_count.cviewer(),
                                     nullptr,
                                     0,
                                     nullptr,
                                     nullptr);
            else
                reference_spmv_dot_kernel<false, false>
                    <<<blocks, 32>>>(scale,
                                     A.cview(),
                                     x.cview(),
                                     y.view(),
                                     dot.viewer(),
                                     triplet_count.cviewer(),
                                     nullptr,
                                     0,
                                     nullptr,
                                     nullptr);
        }
        else if(pipelined)
            Spmv().rbk_sym_spmv_dot_pipelined(scale,
                                              A.cview(),
                                              x.cview(),
                                              y.view(),
                                              dot.view(),
                                              next_y.view(),
                                              next_dot.view(),
                                              converged.view(),
                                              triplet_count.cviewer(),
                                              A.triplet_capacity(),
                                              nullptr);
        else
            Spmv().rbk_sym_spmv_dot(scale,
                                    A.cview(),
                                    x.cview(),
                                    Float{0},
                                    y.view(),
                                    dot.view(),
                                    triplet_count.cviewer(),
                                    A.triplet_capacity(),
                                    nullptr);
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
        y_out.resize(3 * block_rows);
        y.buffer_view().copy_to(y_out.data());
        dot_out = dot;
    };

    std::vector<int>       rows, cols;
    std::vector<Matrix3x3> values;
    int                    block_rows = 0;
    for(Float scale : {Float{1}, Float{0.7}})
        for(bool pipelined : {false, true})
        {
            INFO("scale " << scale << " pipelined " << pipelined);
            // many warps (and a partial last one): y bitwise, pAp to rounding
            make_matrix(4096, 13, rows, cols, values, block_rows);
            std::vector<Float> y_ref, y_new;
            Float              dot_ref = 0, dot_new = 0;
            run(rows, cols, values, block_rows, scale, pipelined, true, y_ref, dot_ref);
            run(rows, cols, values, block_rows, scale, pipelined, false, y_new, dot_new);
            int y_mismatches = 0;
            for(SizeT i = 0; i < y_ref.size(); ++i)
                y_mismatches += std::memcmp(&y_ref[i], &y_new[i], sizeof(Float)) != 0;
            CHECK(y_mismatches == 0);
            REQUIRE(std::isfinite(dot_new));
            CHECK(dot_new == Catch::Approx(dot_ref).epsilon(1e-10));

            // single warps (full and partial): y and pAp bitwise
            int dot_mismatches = 0, single_y_mismatches = 0;
            for(int k = 0; k < 256; ++k)
            {
                make_matrix(1, k % 3 == 0 ? 1 + k % 31 : 32, rows, cols, values, block_rows);
                run(rows, cols, values, block_rows, scale, pipelined, true, y_ref, dot_ref);
                run(rows, cols, values, block_rows, scale, pipelined, false, y_new, dot_new);
                dot_mismatches += std::memcmp(&dot_ref, &dot_new, sizeof(Float)) != 0;
                for(SizeT i = 0; i < y_ref.size(); ++i)
                    single_y_mismatches +=
                        std::memcmp(&y_ref[i], &y_new[i], sizeof(Float)) != 0;
            }
            CHECK(dot_mismatches == 0);
            CHECK(single_y_mismatches == 0);
        }
}
