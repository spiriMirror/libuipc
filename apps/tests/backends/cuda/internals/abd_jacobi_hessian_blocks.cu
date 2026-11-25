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
#include <affine_body/abd_jacobi_matrix.h>

#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
enum BlockResult : int
{
    Direct = 0,
    Swapped,
    SameBody,
    ResultCount,
};

constexpr int BlockRows  = 4;
constexpr int BlockCount = BlockRows * BlockRows;

__global__ void evaluate_streamed_blocks_kernel(ABDJacobi lhs,
                                                Matrix3x3 H,
                                                ABDJacobi rhs,
                                                cuda_tool::BufferView<Matrix3x3> output)
{
    const int block_row = blockIdx.x;
    const int block_col = threadIdx.x;
    if(block_row >= BlockRows || block_col >= BlockRows)
        return;

    const int block_index = block_row * BlockRows + block_col;
    const Matrix3x3 direct = ABDJacobi::JT_H_J_block(lhs.T(), H, rhs, block_row, block_col);
    const Matrix3x3 swapped =
        ABDJacobi::JT_H_J_block(rhs.T(), H.transpose(), lhs, block_row, block_col);

    output(Direct * BlockCount + block_index)   = direct;
    output(Swapped * BlockCount + block_index)  = swapped;
    output(SameBody * BlockCount + block_index) = direct + swapped;
}

void require_blocks_equal(const std::vector<Matrix3x3>& actual,
                          BlockResult                   result,
                          const Matrix12x12&            expected)
{
    for(int block_row = 0; block_row < BlockRows; ++block_row)
    {
        for(int block_col = 0; block_col < BlockRows; ++block_col)
        {
            const int block_index = block_row * BlockRows + block_col;
            const auto& block = actual[static_cast<int>(result) * BlockCount + block_index];
            for(int row = 0; row < 3; ++row)
                for(int col = 0; col < 3; ++col)
                {
                    CAPTURE(result, block_row, block_col, row, col);
                    REQUIRE(block(row, col)
                            == expected(3 * block_row + row, 3 * block_col + col));
                }
        }
    }
}
}  // namespace

TEST_CASE("streamed ABD Hessian blocks match the complete 12x12 construction",
          "[build_solve_focused][abd][hessian_blocks]")
{
    const ABDJacobi lhs{Vector3{0.25, -0.75, 1.5}};
    const ABDJacobi rhs{Vector3{-1.25, 0.5, 0.125}};

    Matrix3x3 H;
    H << 2.0, -0.125, 0.75, 0.375, 3.25, -1.5, -0.625, 0.875, 4.5;

    const Matrix12x12 complete = ABDJacobi::JT_H_J(lhs.T(), H, rhs);
    for(int block_row = 0; block_row < 4; ++block_row)
    {
        for(int block_col = 0; block_col < 4; ++block_col)
        {
            const Matrix3x3 streamed =
                ABDJacobi::JT_H_J_block(lhs.T(), H, rhs, block_row, block_col);
            for(int row = 0; row < 3; ++row)
                for(int col = 0; col < 3; ++col)
                    REQUIRE(streamed(row, col)
                            == complete(3 * block_row + row, 3 * block_col + col));
        }
    }

    const Matrix12x12 same_body = ABDJacobi::JT_H_J(lhs.T(), H, rhs)
                                  + ABDJacobi::JT_H_J(rhs.T(), H.transpose(), lhs);
    for(int block_row = 0; block_row < 4; ++block_row)
    {
        for(int block_col = 0; block_col < 4; ++block_col)
        {
            const Matrix3x3 streamed =
                ABDJacobi::JT_H_J_block(lhs.T(), H, rhs, block_row, block_col)
                + ABDJacobi::JT_H_J_block(rhs.T(), H.transpose(), lhs, block_row, block_col);
            for(int row = 0; row < 3; ++row)
                for(int col = 0; col < 3; ++col)
                    REQUIRE(streamed(row, col)
                            == same_body(3 * block_row + row, 3 * block_col + col));
        }
    }

    SECTION("device block layout and body-order variants")
    {
        cuda_tool::DeviceBuffer<Matrix3x3> device_blocks(ResultCount * BlockCount);
        evaluate_streamed_blocks_kernel<<<BlockRows, BlockRows>>>(
            lhs, H, rhs, device_blocks.view());

        std::vector<Matrix3x3> actual;
        device_blocks.copy_to(actual);

        const Matrix12x12 swapped = ABDJacobi::JT_H_J(rhs.T(), H.transpose(), lhs);
        require_blocks_equal(actual, Direct, complete);
        require_blocks_equal(actual, Swapped, swapped);
        require_blocks_equal(actual, SameBody, same_body);
    }
}
