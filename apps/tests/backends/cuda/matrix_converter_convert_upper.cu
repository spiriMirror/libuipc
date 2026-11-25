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
#include <algorithm/matrix_converter.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc::backend::cuda;

namespace
{
template <typename T, int N>
using Block = typename cuda_tool::DeviceTripletMatrix<T, N>::ValueT;

template <typename T, int N>
struct HostTriplets
{
    int                      rows = 0;
    int                      cols = 0;
    std::vector<int>         row_indices;
    std::vector<int>         col_indices;
    std::vector<Block<T, N>> values;
};

template <typename T, int N>
struct HostBCOO
{
    std::vector<int>         row_indices;
    std::vector<int>         col_indices;
    std::vector<Block<T, N>> values;
};

template <typename T, int N>
void upload(const HostTriplets<T, N>& host, cuda_tool::DeviceTripletMatrix<T, N>& device)
{
    device.resize_discard(host.rows, host.cols, host.values.size());
    if(host.values.empty())
        return;
    device.row_indices().copy_from(host.row_indices.data());
    device.col_indices().copy_from(host.col_indices.data());
    device.values().copy_from(host.values.data());
}

template <typename T, int N>
HostBCOO<T, N> download(const cuda_tool::DeviceBCOOMatrix<T, N>& device)
{
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    HostBCOO<T, N> host;
    host.row_indices.resize(device.triplet_count());
    host.col_indices.resize(device.triplet_count());
    host.values.resize(device.triplet_count());
    if(device.triplet_count() > 0)
    {
        device.row_indices().copy_to(host.row_indices.data());
        device.col_indices().copy_to(host.col_indices.data());
        device.values().copy_to(host.values.data());
    }
    return host;
}

// The reference path: compact the upper triangle in place, then convert.
template <typename T, int N>
HostBCOO<T, N> convert_reference(const HostTriplets<T, N>& host)
{
    cuda_tool::DeviceTripletMatrix<T, N> from;
    cuda_tool::DeviceBCOOMatrix<T, N>    to;
    MatrixConverter<T, N>                converter;
    upload(host, from);
    converter.ge2sym(from);
    converter.convert(from, to);
    REQUIRE(to.rows() == host.rows);
    REQUIRE(to.cols() == host.cols);
    return download(to);
}

template <typename T, int N>
void require_bitwise_equal(const HostBCOO<T, N>& actual, const HostBCOO<T, N>& expected)
{
    REQUIRE(actual.row_indices == expected.row_indices);
    REQUIRE(actual.col_indices == expected.col_indices);
    REQUIRE(actual.values.size() == expected.values.size());
    if(!actual.values.empty())
        REQUIRE(std::memcmp(actual.values.data(),
                            expected.values.data(),
                            actual.values.size() * sizeof(Block<T, N>))
                == 0);
}

// Random general triplets: upper and lower blocks, duplicates, (-1, j),
// (i, -1) and zero-valued (-1, -1) slots, shuffled. Every nonzero key repeats
// at most 33 times, so none of its segments spans three warps of the segmented
// reduction and both paths add every block in the same order (a segment that
// spans two warps adds two partial sums to zero, which commutes exactly).
HostTriplets<double, 3> random_triplets(int rows, int keys, int empty_slots, unsigned seed)
{
    std::mt19937                           rng{seed};
    std::uniform_int_distribution<int>     row_dist{0, rows - 1};
    std::uniform_int_distribution<int>     repeat_dist{1, 33};
    std::uniform_real_distribution<double> value_dist{-1.0, 1.0};

    HostTriplets<double, 3> host;
    host.rows = rows;
    host.cols = rows;
    std::vector<std::array<int, 2>> ij;
    for(int k = 0; k < keys; ++k)
    {
        int i = row_dist(rng);
        int j = row_dist(rng);
        if(k % 97 == 0)
            i = -1;  // kept by ge2sym (-1 <= j)
        else if(k % 89 == 0)
            j = -1;  // dropped by ge2sym (i > -1)
        else if(k % 3 == 0)
            j = i;  // diagonal
        const int repeat = k % 5 == 0 ? repeat_dist(rng) : 1 + k % 3;
        for(int r = 0; r < repeat; ++r)
            ij.push_back({i, j});
    }
    // unordered duplicates of earlier keys
    const auto distinct = ij.size();
    for(size_t k = 0; k < distinct / 64; ++k)
        ij.push_back(ij[k * 17 % distinct]);

    std::vector<std::array<int, 2>> capped;
    {
        auto sorted = ij;
        std::sort(sorted.begin(), sorted.end());
        for(size_t b = 0; b < sorted.size();)
        {
            size_t e = b;
            while(e < sorted.size() && sorted[e] == sorted[b])
                ++e;
            for(size_t r = b; r < std::min(e, b + 33); ++r)
                capped.push_back(sorted[r]);
            b = e;
        }
    }
    // unused (-1, -1) slots hold zero blocks, as GlobalLinearSystem leaves
    // them; their sum is zero in any order
    for(int s = 0; s < empty_slots; ++s)
        capped.push_back({-1, -1});
    std::shuffle(capped.begin(), capped.end(), rng);

    for(auto&& [i, j] : capped)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        if(i != -1 || j != -1)
        {
            for(int a = 0; a < 3; ++a)
                for(int b = 0; b < 3; ++b)
                    H(a, b) = value_dist(rng);
        }
        host.row_indices.push_back(i);
        host.col_indices.push_back(j);
        host.values.push_back(H);
    }
    return host;
}
}  // namespace

TEST_CASE("matrix converter tail key sorts after every packed key",
          "[build_solve_focused][matrix_converter][convert_upper]")
{
    // rows + 1 that fits the row field keeps the sort width
    constexpr auto config = matrix_converter_radix_key_config(8192, 4097);
    STATIC_REQUIRE(matrix_converter_radix_tail_key(config)
                   > matrix_converter_pack_radix_key(-1, -1, config));
    STATIC_REQUIRE(matrix_converter_radix_tail_end_bit(config) == config.end_bit);
    STATIC_REQUIRE((matrix_converter_radix_tail_key(config)
                    >> matrix_converter_radix_tail_end_bit(config))
                   == 0);

    // rows + 1 that is a power of two widens the sort by one bit
    constexpr auto wide = matrix_converter_radix_key_config(8191, 4097);
    STATIC_REQUIRE(wide.end_bit == 26);
    STATIC_REQUIRE(matrix_converter_radix_tail_end_bit(wide) == 27);
    STATIC_REQUIRE(matrix_converter_radix_tail_key(wide)
                   > matrix_converter_pack_radix_key(-1, -1, wide));
    STATIC_REQUIRE((matrix_converter_radix_tail_key(wide)
                    >> matrix_converter_radix_tail_end_bit(wide))
                   == 0);

    // all-ones column field: the tail key must not equal (-1, -1)
    constexpr auto ones = matrix_converter_radix_key_config(7, 7);
    STATIC_REQUIRE(ones.col_mask == 7);
    STATIC_REQUIRE(matrix_converter_radix_tail_key(ones)
                   > matrix_converter_pack_radix_key(-1, -1, ones));
    STATIC_REQUIRE((matrix_converter_radix_tail_key(ones)
                    >> matrix_converter_radix_tail_end_bit(ones))
                   == 0);

    constexpr auto max_config =
        matrix_converter_radix_key_config(std::numeric_limits<int>::max(),
                                          std::numeric_limits<int>::max());
    STATIC_REQUIRE(matrix_converter_radix_tail_end_bit(max_config) == 63);
    STATIC_REQUIRE(matrix_converter_radix_tail_key(max_config)
                   > matrix_converter_pack_radix_key(-1, -1, max_config));
}

TEST_CASE("matrix converter convert_upper matches ge2sym then convert on edge cases",
          "[build_solve_focused][matrix_converter][convert_upper]")
{
    // rows = 3 gives an all-ones row field: (-1, -1) has the largest packed key
    HostTriplets<float, 1> host;
    host.rows        = 3;
    host.cols        = 3;
    host.row_indices = {2, 0, -1, 1, 2, 0, -1, 0, 1, 2, -1, 0, 2};
    host.col_indices = {0, 2, -1, 1, 2, 2, 1, -1, 0, 2, -1, 0, -1};
    host.values = {1.0f, 2.0f, 0.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f, 256.0f, 0.0f, 512.0f, 1024.0f};

    cuda_tool::DeviceTripletMatrix<float, 1> from;
    cuda_tool::DeviceBCOOMatrix<float, 1>    to;
    MatrixConverter<float, 1>                converter;
    upload(host, from);
    converter.convert_upper(from, to);
    REQUIRE(to.rows() == 3);
    REQUIRE(to.cols() == 3);
    const auto actual = download(to);

    // (0,0) (0,2)x2 (1,1) (2,2)x2 (-1,1) (-1,-1)x2; lower and (i,-1) dropped
    REQUIRE(actual.row_indices == std::vector<int>{0, 0, 1, 2, -1, -1});
    REQUIRE(actual.col_indices == std::vector<int>{0, 2, 1, 2, 1, -1});
    REQUIRE(actual.values == std::vector<float>{512.0f, 18.0f, 4.0f, 264.0f, 32.0f, 0.0f});
    require_bitwise_equal(actual, convert_reference(host));

    // the input is not compacted
    REQUIRE(from.triplet_count() == host.values.size());

    // only lower blocks: an empty result, then reuse with a new shape
    HostTriplets<float, 1> lower;
    lower.rows        = 4;
    lower.cols        = 4;
    lower.row_indices = {3, 1, 2, 3};
    lower.col_indices = {0, 0, -1, 2};
    lower.values      = {1.0f, 2.0f, 4.0f, 8.0f};
    upload(lower, from);
    converter.convert_upper(from, to);
    REQUIRE(to.rows() == 4);
    REQUIRE(to.triplet_count() == 0);
    require_bitwise_equal(download(to), convert_reference(lower));

    // no lower blocks: the upper prefix is the whole input
    HostTriplets<float, 1> upper;
    upper.rows        = 5;
    upper.cols        = 5;
    upper.row_indices = {4, 0, 4, 1};
    upper.col_indices = {4, 3, 4, 1};
    upper.values      = {1.0f, 2.0f, 4.0f, 8.0f};
    upload(upper, from);
    converter.convert_upper(from, to);
    require_bitwise_equal(download(to), convert_reference(upper));
    REQUIRE(to.triplet_count() == 3);

    // empty input publishes the shape and no blocks
    from.resize_discard(6, 6, 0);
    converter.convert_upper(from, to);
    REQUIRE(to.rows() == 6);
    REQUIRE(to.cols() == 6);
    REQUIRE(to.triplet_count() == 0);
}

TEST_CASE("matrix converter convert_upper is bitwise ge2sym then convert on 3x3 blocks",
          "[build_solve_focused][matrix_converter][convert_upper]")
{
    MatrixConverter<double, 3>                converter;
    cuda_tool::DeviceTripletMatrix<double, 3> from;
    cuda_tool::DeviceBCOOMatrix<double, 3>    to;

    // two sizes through one converter: growth and shrink of its scratch
    for(auto&& [rows, keys, empty_slots, seed] :
        {std::array<int, 4>{20000, 60000, 5000, 7}, std::array<int, 4>{700, 3000, 0, 11}})
    {
        const auto host = random_triplets(rows, keys, empty_slots, (unsigned)seed);
        upload(host, from);
        converter.convert_upper(from, to);
        const auto actual = download(to);
        REQUIRE(actual.values.size() > 0);
        require_bitwise_equal(actual, convert_reference(host));
    }
}
