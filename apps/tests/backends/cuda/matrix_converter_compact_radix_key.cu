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
#include <cuda_tool/cub.h>

#include <array>
#include <limits>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc::backend::cuda;

TEST_CASE("matrix converter compact radix key preserves lexicographic order",
          "[build_solve_focused][matrix_converter][compact_radix_key]")
{
    constexpr int Rows = 8192;
    constexpr int Cols = 4097;

    constexpr auto config = matrix_converter_radix_key_config(Rows, Cols);
    STATIC_REQUIRE(config.col_bits == 13);
    STATIC_REQUIRE(config.end_bit == 27);
    STATIC_REQUIRE(config.compact);
    STATIC_REQUIRE(config.rows == Rows);
    STATIC_REQUIRE(config.cols == Cols);

    STATIC_REQUIRE(matrix_converter_radix_index_is_valid(-1, Rows));
    STATIC_REQUIRE(matrix_converter_radix_index_is_valid(0, Rows));
    STATIC_REQUIRE(matrix_converter_radix_index_is_valid(Rows - 1, Rows));
    STATIC_REQUIRE_FALSE(matrix_converter_radix_index_is_valid(-2, Rows));
    STATIC_REQUIRE_FALSE(matrix_converter_radix_index_is_valid(Rows, Rows));

    constexpr auto empty_config = matrix_converter_radix_key_config(0, Cols);
    STATIC_REQUIRE_FALSE(empty_config.compact);
    STATIC_REQUIRE(empty_config.end_bit == 64);

    constexpr auto max_config =
        matrix_converter_radix_key_config(std::numeric_limits<int>::max(),
                                          std::numeric_limits<int>::max());
    STATIC_REQUIRE(max_config.compact);
    STATIC_REQUIRE(max_config.end_bit == 62);
    constexpr auto max_key =
        matrix_converter_pack_radix_key(std::numeric_limits<int>::max() - 1,
                                        std::numeric_limits<int>::max() - 1,
                                        max_config);
    constexpr auto max_ij = matrix_converter_unpack_radix_key(max_key, max_config);
    STATIC_REQUIRE(max_ij.x == std::numeric_limits<int>::max() - 1);
    STATIC_REQUIRE(max_ij.y == std::numeric_limits<int>::max() - 1);
    constexpr auto max_sentinel_key = matrix_converter_pack_radix_key(-1, -1, max_config);
    STATIC_REQUIRE((matrix_converter_unpack_radix_key(max_sentinel_key, max_config)
                    == MatrixConverterIntPair{-1, -1}));

    constexpr auto valid_last_key =
        matrix_converter_pack_radix_key(Rows - 1, Cols - 1, config);
    constexpr auto col_sentinel_key =
        matrix_converter_pack_radix_key(Rows - 1, -1, config);
    constexpr auto row_sentinel_key = matrix_converter_pack_radix_key(-1, 0, config);
    constexpr auto both_sentinel_key = matrix_converter_pack_radix_key(-1, -1, config);
    STATIC_REQUIRE(valid_last_key < col_sentinel_key);
    STATIC_REQUIRE(col_sentinel_key < row_sentinel_key);
    STATIC_REQUIRE(row_sentinel_key < both_sentinel_key);
    STATIC_REQUIRE((matrix_converter_unpack_radix_key(col_sentinel_key, config)
                    == MatrixConverterIntPair{Rows - 1, -1}));
    STATIC_REQUIRE((matrix_converter_unpack_radix_key(row_sentinel_key, config)
                    == MatrixConverterIntPair{-1, 0}));
    STATIC_REQUIRE((matrix_converter_unpack_radix_key(both_sentinel_key, config)
                    == MatrixConverterIntPair{-1, -1}));

    const std::array<int, 8> rows = {8191, 0, 17, 17, 4096, 0, 17, 8191};
    const std::array<int, 8> cols = {4096, 1, 2, 2, 0, 0, 4096, 0};
    const std::array<float, 8> values = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f};

    cuda_tool::DeviceTripletMatrix<float, 1> from;
    cuda_tool::DeviceBCOOMatrix<float, 1>    to;
    MatrixConverter<float, 1>                converter;

    from.resize(Rows, Cols, rows.size());
    from.row_indices().copy_from(rows.data());
    from.col_indices().copy_from(cols.data());
    from.values().copy_from(values.data());

    converter.convert(from, to);

    std::vector<int>   actual_rows(to.triplet_count());
    std::vector<int>   actual_cols(to.triplet_count());
    std::vector<float> actual_values(to.triplet_count());
    to.row_indices().copy_to(actual_rows.data());
    to.col_indices().copy_to(actual_cols.data());
    to.values().copy_to(actual_values.data());

    const std::vector<int> expected_rows = {0, 0, 17, 17, 4096, 8191, 8191};
    const std::vector<int> expected_cols = {0, 1, 2, 4096, 0, 0, 4096};
    const std::vector<float> expected_values = {32.0f, 2.0f, 12.0f, 64.0f, 16.0f, 128.0f, 1.0f};

    REQUIRE(actual_rows == expected_rows);
    REQUIRE(actual_cols == expected_cols);
    REQUIRE(actual_values == expected_values);

    for(size_t i = 0; i < rows.size(); ++i)
    {
        const auto key = matrix_converter_pack_radix_key(rows[i], cols[i], config);
        const auto ij = matrix_converter_unpack_radix_key(key, config);
        REQUIRE(ij.x == rows[i]);
        REQUIRE(ij.y == cols[i]);
    }
}

TEST_CASE("matrix converter compact radix key preserves and orders sentinels",
          "[build_solve_focused][matrix_converter][compact_radix_key]")
{
    constexpr int Rows = 2;
    constexpr int Cols = 2;

    const std::array<int, 6>   rows   = {1, -1, 0, 1, 0, -1};
    const std::array<int, 6>   cols   = {1, -1, -1, -1, 0, 0};
    const std::array<float, 6> values = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f};

    cuda_tool::DeviceBCOOMatrix<float, 1> matrix;
    MatrixConverter<float, 1>             converter;
    matrix.resize(Rows, Cols, rows.size());
    matrix.row_indices().copy_from(rows.data());
    matrix.col_indices().copy_from(cols.data());
    matrix.values().copy_from(values.data());

    converter._radix_sort_indices_and_blocks(matrix);

    std::array<int, 6>   actual_rows{};
    std::array<int, 6>   actual_cols{};
    std::array<float, 6> actual_values{};
    matrix.row_indices().copy_to(actual_rows.data());
    matrix.col_indices().copy_to(actual_cols.data());
    matrix.values().copy_to(actual_values.data());

    const std::array<int, 6> expected_rows = {0, 0, 1, 1, -1, -1};
    const std::array<int, 6> expected_cols = {0, -1, 1, -1, 0, -1};
    const std::array<float, 6> expected_values = {16.0f, 4.0f, 1.0f, 8.0f, 32.0f, 2.0f};

    REQUIRE(actual_rows == expected_rows);
    REQUIRE(actual_cols == expected_cols);
    REQUIRE(actual_values == expected_values);
}

TEST_CASE("triplet matrix radix kernels preserve and order sentinels",
          "[build_solve_focused][matrix_converter][compact_radix_key]")
{
    constexpr int  Rows   = 2;
    constexpr int  Cols   = 2;
    constexpr auto config = matrix_converter_radix_key_config(Rows, Cols);

    const std::array<int, 6> rows = {1, -1, 0, 1, 0, -1};
    const std::array<int, 6> cols = {1, -1, -1, -1, 0, 0};

    cuda_tool::DeviceTripletMatrix<float, 1> from;
    from.resize(Rows, Cols, rows.size());
    from.row_indices().copy_from(rows.data());
    from.col_indices().copy_from(cols.data());

    cuda_tool::DeviceBuffer<uint64_t>               keys_in(rows.size());
    cuda_tool::DeviceBuffer<uint64_t>               keys_out(rows.size());
    cuda_tool::DeviceBuffer<int>                    order_in(rows.size());
    cuda_tool::DeviceBuffer<int>                    order_out(rows.size());
    cuda_tool::DeviceBuffer<MatrixConverterIntPair> pairs(rows.size());

    matrix_converter_radix_sort_indices_and_blocks_k1_kernel<<<1, 32>>>(
        from.row_indices(), from.col_indices(), keys_in.view(), order_in.view(), config, rows.size());
    cuda_tool::DeviceRadixSort().SortPairs(keys_in.data(),
                                           keys_out.data(),
                                           order_in.data(),
                                           order_out.data(),
                                           keys_in.size(),
                                           0,
                                           config.end_bit);
    matrix_converter_radix_sort_indices_and_blocks_k2_kernel<<<1, 32>>>(
        keys_out.view(), pairs.view(), config, rows.size());

    std::array<MatrixConverterIntPair, 6> actual_pairs{};
    std::array<int, 6>                    actual_order{};
    pairs.view().copy_to(actual_pairs.data());
    order_out.view().copy_to(actual_order.data());

    const std::array<MatrixConverterIntPair, 6> expected_pairs = {
        MatrixConverterIntPair{0, 0},
        MatrixConverterIntPair{0, -1},
        MatrixConverterIntPair{1, 1},
        MatrixConverterIntPair{1, -1},
        MatrixConverterIntPair{-1, 0},
        MatrixConverterIntPair{-1, -1}};
    const std::array<int, 6> expected_order = {4, 2, 0, 3, 5, 1};

    REQUIRE(actual_pairs == expected_pairs);
    REQUIRE(actual_order == expected_order);
}

TEST_CASE("triplet matrix conversion preserves sentinel runs",
          "[build_solve_focused][matrix_converter][compact_radix_key]")
{
    constexpr int Rows = 2;
    constexpr int Cols = 2;

    const std::array<int, 8> rows = {1, -1, 0, 1, 0, -1, 0, -1};
    const std::array<int, 8> cols = {1, -1, -1, -1, 0, 0, -1, -1};
    const std::array<float, 8> values = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f};

    cuda_tool::DeviceTripletMatrix<float, 1> from;
    cuda_tool::DeviceBCOOMatrix<float, 1>    to;
    MatrixConverter<float, 1>                converter;
    from.resize(Rows, Cols, rows.size());
    from.row_indices().copy_from(rows.data());
    from.col_indices().copy_from(cols.data());
    from.values().copy_from(values.data());

    converter.convert(from, to);

    std::vector<int>   actual_rows(to.triplet_count());
    std::vector<int>   actual_cols(to.triplet_count());
    std::vector<float> actual_values(to.triplet_count());
    to.row_indices().copy_to(actual_rows.data());
    to.col_indices().copy_to(actual_cols.data());
    to.values().copy_to(actual_values.data());

    const std::vector<int> expected_rows = {0, 0, 1, 1, -1, -1};
    const std::vector<int> expected_cols = {0, -1, 1, -1, 0, -1};
    const std::vector<float> expected_values = {16.0f, 68.0f, 1.0f, 8.0f, 32.0f, 130.0f};

    REQUIRE(actual_rows == expected_rows);
    REQUIRE(actual_cols == expected_cols);
    REQUIRE(actual_values == expected_values);
}

TEST_CASE("compact radix sort remains stable for duplicate matrix keys",
          "[build_solve_focused][matrix_converter][compact_radix_key]")
{
    constexpr auto config = matrix_converter_radix_key_config(32, 17);
    const std::array<MatrixConverterIntPair, 8> ij = {MatrixConverterIntPair{3, 4},
                                                      MatrixConverterIntPair{1, 2},
                                                      MatrixConverterIntPair{3, 4},
                                                      MatrixConverterIntPair{0, 16},
                                                      MatrixConverterIntPair{1, 2},
                                                      MatrixConverterIntPair{31, 0},
                                                      MatrixConverterIntPair{3, 4},
                                                      MatrixConverterIntPair{0, 0}};

    std::array<uint64_t, ij.size()> keys{};
    std::array<int, ij.size()>      order{};
    for(size_t i = 0; i < ij.size(); ++i)
    {
        keys[i]  = matrix_converter_pack_radix_key(ij[i].x, ij[i].y, config);
        order[i] = static_cast<int>(i);
    }

    cuda_tool::DeviceBuffer<uint64_t> keys_in(keys.size());
    cuda_tool::DeviceBuffer<uint64_t> keys_out(keys.size());
    cuda_tool::DeviceBuffer<int>      order_in(order.size());
    cuda_tool::DeviceBuffer<int>      order_out(order.size());
    keys_in.view().copy_from(keys.data());
    order_in.view().copy_from(order.data());

    cuda_tool::DeviceRadixSort().SortPairs(keys_in.data(),
                                           keys_out.data(),
                                           order_in.data(),
                                           order_out.data(),
                                           keys.size(),
                                           0,
                                           config.end_bit);

    std::array<int, order.size()> actual{};
    order_out.view().copy_to(actual.data());
    const std::array<int, order.size()> expected = {7, 3, 1, 4, 0, 2, 6, 5};
    REQUIRE(actual == expected);
}

TEST_CASE("matrix converter compact radix key sorts BCOO in place",
          "[build_solve_focused][matrix_converter][compact_radix_key]")
{
    constexpr int Rows = 8192;
    constexpr int Cols = 4097;

    const std::array<int, 8> rows = {8191, 0, 17, 17, 4096, 0, 17, 8191};
    const std::array<int, 8> cols = {4096, 1, 2, 2, 0, 0, 4096, 0};
    const std::array<float, 8> values = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f};

    cuda_tool::DeviceBCOOMatrix<float, 1> matrix;
    MatrixConverter<float, 1>             converter;
    matrix.resize(Rows, Cols, rows.size());
    matrix.row_indices().copy_from(rows.data());
    matrix.col_indices().copy_from(cols.data());
    matrix.values().copy_from(values.data());

    converter._radix_sort_indices_and_blocks(matrix);

    std::array<int, 8>   actual_rows{};
    std::array<int, 8>   actual_cols{};
    std::array<float, 8> actual_values{};
    matrix.row_indices().copy_to(actual_rows.data());
    matrix.col_indices().copy_to(actual_cols.data());
    matrix.values().copy_to(actual_values.data());

    const std::array<int, 8> expected_rows = {0, 0, 17, 17, 17, 4096, 8191, 8191};
    const std::array<int, 8>   expected_cols   = {0, 1, 2, 2, 4096, 0, 0, 4096};
    const std::array<float, 8> expected_values = {
        32.0f, 2.0f, 4.0f, 8.0f, 64.0f, 16.0f, 128.0f, 1.0f};

    REQUIRE(actual_rows == expected_rows);
    REQUIRE(actual_cols == expected_cols);
    REQUIRE(actual_values == expected_values);
}

TEST_CASE("matrix converter clears empty output and reuses storage across shapes",
          "[build_solve_focused][matrix_converter][compact_radix_key][empty_conversion]")
{
    cuda_tool::DeviceTripletMatrix<float, 1> from;
    cuda_tool::DeviceBCOOMatrix<float, 1>    to;
    MatrixConverter<float, 1>                converter;

    auto convert_and_check = [&](int                       rows,
                                 int                       cols,
                                 const std::vector<int>&   expected_rows,
                                 const std::vector<int>&   expected_cols,
                                 const std::vector<float>& expected_values)
    {
        converter.convert(from, to);
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());
        REQUIRE(to.rows() == rows);
        REQUIRE(to.cols() == cols);
        REQUIRE(to.triplet_count() == expected_values.size());
        REQUIRE(to.row_indices().size() == expected_rows.size());
        REQUIRE(to.col_indices().size() == expected_cols.size());
        REQUIRE(to.values().size() == expected_values.size());

        std::vector<int>   actual_rows(to.triplet_count());
        std::vector<int>   actual_cols(to.triplet_count());
        std::vector<float> actual_values(to.triplet_count());
        if(!expected_values.empty())
        {
            to.row_indices().copy_to(actual_rows.data());
            to.col_indices().copy_to(actual_cols.data());
            to.values().copy_to(actual_values.data());
        }
        REQUIRE(actual_rows == expected_rows);
        REQUIRE(actual_cols == expected_cols);
        REQUIRE(actual_values == expected_values);
    };

    // A real conversion into a fresh output must publish the empty shape.
    from.resize(3, 5, 0);
    convert_and_check(3, 5, {}, {}, {});

    {
        const std::array<int, 5>   rows   = {3, 0, 3, 1, 0};
        const std::array<int, 5>   cols   = {4, 2, 4, 0, 2};
        const std::array<float, 5> values = {2.0f, 4.0f, 8.0f, 16.0f, 32.0f};
        from.resize(4, 5, rows.size());
        from.row_indices().copy_from(rows.data());
        from.col_indices().copy_from(cols.data());
        from.values().copy_from(values.data());
        convert_and_check(4, 5, {0, 1, 3}, {2, 0, 4}, {36.0f, 16.0f, 10.0f});
    }

    // Keep the same converter, input, and output: empty conversion must clear
    // the previous three merged entries even if their capacity is retained.
    from.resize(4, 5, 0);
    convert_and_check(4, 5, {}, {}, {});
    from.resize(0, 7, 0);
    convert_and_check(0, 7, {}, {}, {});
    from.resize(3, 2, 0);
    convert_and_check(3, 2, {}, {}, {});

    {
        const std::array<int, 4>   rows   = {2, 0, 2, 1};
        const std::array<int, 4>   cols   = {1, 0, 1, 0};
        const std::array<float, 4> values = {1.0f, 2.0f, 4.0f, 8.0f};
        from.resize(3, 2, rows.size());
        from.row_indices().copy_from(rows.data());
        from.col_indices().copy_from(cols.data());
        from.values().copy_from(values.data());
        convert_and_check(3, 2, {0, 1, 2}, {0, 0, 1}, {2.0f, 8.0f, 5.0f});
    }
}

TEST_CASE("matrix converter sym2ge counts the diagonals that are actually stored",
          "[build_solve_focused][matrix_converter][sym2ge]")
{
    constexpr int Rows = 4;
    constexpr int Cols = 4;

    // A valid upper-triangular BCOO is not required to store one diagonal
    // block per row.  This matrix has one diagonal and four off-diagonals.
    const std::array<int, 5>       rows = {0, 0, 0, 1, 2};
    const std::array<int, 5>       cols = {0, 1, 2, 3, 3};
    std::array<Eigen::Matrix3f, 5> values;
    values[0] = Eigen::Matrix3f::Identity();
    for(size_t i = 1; i < values.size(); ++i)
    {
        for(int row = 0; row < 3; ++row)
        {
            for(int col = 0; col < 3; ++col)
                values[i](row, col) = float(100 * i + 10 * row + col);
        }
    }

    cuda_tool::DeviceBCOOMatrix<float, 3> from;
    cuda_tool::DeviceBCOOMatrix<float, 3> to;
    MatrixConverter<float, 3>             converter;
    from.resize(Rows, Cols, rows.size());
    from.row_indices().copy_from(rows.data());
    from.col_indices().copy_from(cols.data());
    from.values().copy_from(values.data());

    converter.sym2ge(from, to);
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    REQUIRE(to.rows() == Rows);
    REQUIRE(to.cols() == Cols);
    REQUIRE(to.triplet_count() == 9);

    std::array<int, 9>             actual_rows{};
    std::array<int, 9>             actual_cols{};
    std::array<Eigen::Matrix3f, 9> actual_values;
    to.row_indices().copy_to(actual_rows.data());
    to.col_indices().copy_to(actual_cols.data());
    to.values().copy_to(actual_values.data());

    const std::array<int, 9> expected_rows = {0, 0, 0, 1, 1, 2, 2, 3, 3};
    const std::array<int, 9> expected_cols = {0, 1, 2, 0, 3, 0, 3, 1, 2};
    const std::array<Eigen::Matrix3f, 9> expected_values = {values[0],
                                                            values[1],
                                                            values[2],
                                                            values[1].transpose(),
                                                            values[3],
                                                            values[2].transpose(),
                                                            values[4],
                                                            values[3].transpose(),
                                                            values[4].transpose()};

    REQUIRE(actual_rows == expected_rows);
    REQUIRE(actual_cols == expected_cols);
    for(size_t i = 0; i < actual_values.size(); ++i)
        REQUIRE(actual_values[i] == expected_values[i]);
}

TEST_CASE("matrix converter sym2ge accepts a nonempty matrix without diagonals",
          "[build_solve_focused][matrix_converter][sym2ge]")
{
    const std::array<int, 2>             rows   = {0, 1};
    const std::array<int, 2>             cols   = {1, 2};
    const std::array<Eigen::Matrix3f, 2> values = {Eigen::Matrix3f::Identity(),
                                                   2.0f * Eigen::Matrix3f::Identity()};

    cuda_tool::DeviceBCOOMatrix<float, 3> from;
    cuda_tool::DeviceBCOOMatrix<float, 3> to;
    MatrixConverter<float, 3>             converter;
    from.resize(3, 3, rows.size());
    from.row_indices().copy_from(rows.data());
    from.col_indices().copy_from(cols.data());
    from.values().copy_from(values.data());

    converter.sym2ge(from, to);
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    REQUIRE(to.rows() == 3);
    REQUIRE(to.cols() == 3);
    REQUIRE(to.triplet_count() == 4);
}

TEST_CASE("matrix converter sym2ge preserves a shaped empty matrix",
          "[build_solve_focused][matrix_converter][sym2ge][empty_conversion]")
{
    cuda_tool::DeviceBCOOMatrix<float, 3> from;
    cuda_tool::DeviceBCOOMatrix<float, 3> to;
    MatrixConverter<float, 3>             converter;

    const std::array<int, 1>             rows   = {0};
    const std::array<int, 1>             cols   = {1};
    const std::array<Eigen::Matrix3f, 1> values = {Eigen::Matrix3f::Identity()};
    from.resize(2, 2, 1);
    from.row_indices().copy_from(rows.data());
    from.col_indices().copy_from(cols.data());
    from.values().copy_from(values.data());
    converter.sym2ge(from, to);
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    REQUIRE(to.triplet_count() == 2);

    from.resize(4, 7, 0);
    converter.sym2ge(from, to);
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    REQUIRE(to.rows() == 4);
    REQUIRE(to.cols() == 7);
    REQUIRE(to.triplet_count() == 0);
}
