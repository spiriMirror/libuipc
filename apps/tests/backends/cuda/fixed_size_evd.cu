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

// type_define.h must come before anything that includes Eigen (here
// contact_type_block_layout.h): it declares the global ::arg that Eigen's
// `using ::arg` needs in nvcc's device pass, and Windows builds reject Eigen
// when ::arg is declared after it.
#include <type_define.h>
#include <utils/contact_type_block_layout.h>
#include <utils/fixed_bank_soa_evd.h>
#include <cuda_tool/stream.h>

#include "eigen_reference.h"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
template <typename T>
class DeviceAllocation
{
  public:
    explicit DeviceAllocation(std::size_t count)
        : m_count{count}
    {
        CUDA_TOOL_CHECK(cudaMalloc(&m_data, count * sizeof(T)));
    }

    DeviceAllocation(const DeviceAllocation&)            = delete;
    DeviceAllocation& operator=(const DeviceAllocation&) = delete;

    ~DeviceAllocation()
    {
        if(m_data)
            cudaFree(m_data);
    }

    T* data() const { return m_data; }

  private:
    T*          m_data  = nullptr;
    std::size_t m_count = 0;
};

template <int N, int LanePitch>
__global__ void fixed_size_evd_test_kernel(const Float* input,
                                           Float*       output_psd,
                                           Float*       output_eigenvalues,
                                           int*         output_status,
                                           int          count)
{
    __shared__ Float shared_workspace[N * N * LanePitch];

    const int matrix_id = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if(matrix_id >= count)
        return;

    FixedBankSoAMap<N, LanePitch> workspace(shared_workspace + threadIdx.x);
    const std::size_t input_offset = static_cast<std::size_t>(matrix_id) * N * N;

#pragma unroll
    for(int col = 0; col < N; ++col)
    {
#pragma unroll
        for(int row = 0; row < N; ++row)
        {
            workspace(row, col) =
                row >= col ? input[input_offset + col * N + row] : Float(0);
        }
    }

    Vector<Float, N> eigenvalues;
    if(!selfadjoint_evd_fixed_bank_shared<N>(workspace, eigenvalues))
    {
        output_status[matrix_id] = 1;
        return;
    }

    bool finite = true;
#pragma unroll
    for(int k = 0; k < N; ++k)
    {
        output_eigenvalues[static_cast<std::size_t>(matrix_id) * N + k] = eigenvalues(k);
        finite = finite && isfinite(eigenvalues(k));
    }

#pragma unroll
    for(int col = 0; col < N; ++col)
    {
#pragma unroll
        for(int row = 0; row < N; ++row)
        {
            Float value = 0;
#pragma unroll
            for(int k = 0; k < N; ++k)
            {
                value = fma(workspace(row, k), eigenvalues(k) * workspace(col, k), value);
            }
            output_psd[input_offset + col * N + row] = value;
            finite = finite && isfinite(value);
        }
    }

    output_status[matrix_id] = finite ? 0 : 2;
}

template <int N, int BlockSize, int LanePitch>
void run_fixed_size_evd_test(int count)
{
    static_assert(std::is_same_v<Float, double>);
    static_assert(BlockSize > 0 && BlockSize <= LanePitch);

    using MatrixN = Eigen::Matrix<Float, N, N>;
    using VectorN = Eigen::Matrix<Float, N, 1>;

    std::mt19937_64 generator(0x5eed0000ULL + N * 1000 + LanePitch * 10 + count);
    std::uniform_real_distribution<Float> distribution(-1.0, 1.0);

    std::vector<Float>   input(static_cast<std::size_t>(count) * N * N);
    std::vector<MatrixN> expected_psd(count);
    std::vector<VectorN> expected_eigenvalues(count);

    for(int matrix_id = 0; matrix_id < count; ++matrix_id)
    {
        MatrixN matrix;
        for(int col = 0; col < N; ++col)
        {
            for(int row = 0; row < N; ++row)
                matrix(row, col) = distribution(generator);
        }
        matrix = ((matrix + matrix.transpose()) * Float(0.5)).eval();

        if(matrix_id == 0)
        {
            matrix.setZero();
        }
        else if(matrix_id == 1)
        {
            matrix.setZero();
            for(int i = 0; i < N; ++i)
            {
                matrix(i, i) = i < N / 3 ? Float(-2) : Float(1);  // repeated eigenvalues
            }
        }
        else if(matrix_id == 2)
        {
            matrix *= Float(1e-12);
        }
        else if(matrix_id == 3)
        {
            matrix *= Float(1e6);
        }

        const std::size_t offset = static_cast<std::size_t>(matrix_id) * N * N;
        for(int col = 0; col < N; ++col)
        {
            for(int row = 0; row < N; ++row)
                input[offset + col * N + row] = matrix(row, col);
        }

        VectorN eigenvalues;
        MatrixN eigenvectors;
        REQUIRE(uipc::test_reference::symmetric_eigen(
            N, matrix.data(), eigenvalues.data(), eigenvectors.data()));
        expected_eigenvalues[matrix_id] = eigenvalues.cwiseMax(Float(0));
        expected_psd[matrix_id]         = eigenvectors
                                  * expected_eigenvalues[matrix_id].asDiagonal()
                                  * eigenvectors.transpose();
    }

    DeviceAllocation<Float> device_input(input.size());
    DeviceAllocation<Float> device_psd(input.size());
    DeviceAllocation<Float> device_eigenvalues(static_cast<std::size_t>(count) * N);
    DeviceAllocation<int> device_status(count);

    CUDA_TOOL_CHECK(cudaMemcpy(
        device_input.data(), input.data(), input.size() * sizeof(Float), cudaMemcpyHostToDevice));
    CUDA_TOOL_CHECK(cudaMemset(device_status.data(), 0xff, count * sizeof(int)));

    fixed_size_evd_test_kernel<N, LanePitch>
        <<<(count + BlockSize - 1) / BlockSize, BlockSize>>>(device_input.data(),
                                                             device_psd.data(),
                                                             device_eigenvalues.data(),
                                                             device_status.data(),
                                                             count);
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    std::vector<Float> output_psd(input.size());
    std::vector<Float> output_eigenvalues(static_cast<std::size_t>(count) * N);
    std::vector<int>   output_status(count);
    CUDA_TOOL_CHECK(cudaMemcpy(output_psd.data(),
                               device_psd.data(),
                               output_psd.size() * sizeof(Float),
                               cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(output_eigenvalues.data(),
                               device_eigenvalues.data(),
                               output_eigenvalues.size() * sizeof(Float),
                               cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(output_status.data(),
                               device_status.data(),
                               output_status.size() * sizeof(int),
                               cudaMemcpyDeviceToHost));

    for(int matrix_id = 0; matrix_id < count; ++matrix_id)
    {
        REQUIRE(output_status[matrix_id] == 0);
        const std::size_t matrix_offset = static_cast<std::size_t>(matrix_id) * N * N;
        const std::size_t eigen_offset = static_cast<std::size_t>(matrix_id) * N;

        Eigen::Map<const MatrixN> actual_psd(output_psd.data() + matrix_offset);
        Eigen::Map<const VectorN> actual_eigenvalues(output_eigenvalues.data() + eigen_offset);

        const Float psd_scale =
            std::max<Float>(Float(1), expected_psd[matrix_id].norm());
        const Float eigen_scale =
            std::max<Float>(Float(1), expected_eigenvalues[matrix_id].norm());
        REQUIRE((actual_psd - expected_psd[matrix_id]).norm() / psd_scale <= Float(1e-12));
        REQUIRE((actual_eigenvalues - expected_eigenvalues[matrix_id]).norm() / eigen_scale
                <= Float(1e-12));
        REQUIRE((actual_psd - actual_psd.transpose()).norm() / psd_scale <= Float(1e-12));
    }
}

template <int BlockSize>
void run_production_geometry_evd_tests()
{
    constexpr int LanePitch = BlockSize;
    for(int count : std::array{1, BlockSize - 1, BlockSize, BlockSize + 1})
    {
        INFO("CTA=" << BlockSize << " count=" << count);
        run_fixed_size_evd_test<6, BlockSize, LanePitch>(count);
        run_fixed_size_evd_test<9, BlockSize, LanePitch>(count);
        run_fixed_size_evd_test<12, BlockSize, LanePitch>(count);
    }
}
}  // namespace

TEST_CASE("fixed-bank shared EVD matches CPU Eigen at production launch geometries",
          "[cuda][fixed_size_evd]")
{
    CUDA_TOOL_CHECK(cudaDeviceSetLimit(cudaLimitStackSize, 32 * 1024));
    run_production_geometry_evd_tests<8>();
    run_production_geometry_evd_tests<16>();
    run_production_geometry_evd_tests<32>();
}

template <int BlockSize>
void check_contact_type_block_layout()
{
    for(int pt_count = 0; pt_count <= 2 * BlockSize; ++pt_count)
    {
        for(int ee_count = 0; ee_count <= 2 * BlockSize; ++ee_count)
        {
            for(int pe_count = 0; pe_count <= 2 * BlockSize; ++pe_count)
            {
                for(int pp_count = 0; pp_count <= 2 * BlockSize; ++pp_count)
                {
                    const auto layout = make_contact_type_block_layout<BlockSize>(
                        pt_count, ee_count, pe_count, pp_count);
                    REQUIRE(layout.pt_end == pt_count);
                    REQUIRE(layout.ee_end - layout.ee_offset == ee_count);
                    REQUIRE(layout.pe_end - layout.pe_offset == pe_count);
                    REQUIRE(layout.padded_total - layout.pp_offset == pp_count);
                    REQUIRE(layout.ee_offset % BlockSize == 0);
                    REQUIRE(layout.pe_offset % BlockSize == 0);
                    REQUIRE(layout.pp_offset % BlockSize == 0);

                    for(int block_begin = 0; block_begin < layout.padded_total;
                        block_begin += BlockSize)
                    {
                        int block_type = -1;
                        for(int idx = block_begin;
                            idx < block_begin + BlockSize && idx < layout.padded_total;
                            ++idx)
                        {
                            int type = -1;
                            if(idx < layout.pt_end)
                                type = 0;
                            else if(idx >= layout.ee_offset && idx < layout.ee_end)
                                type = 1;
                            else if(idx >= layout.pe_offset && idx < layout.pe_end)
                                type = 2;
                            else if(idx >= layout.pp_offset)
                                type = 3;

                            if(type >= 0)
                            {
                                if(block_type < 0)
                                    block_type = type;
                                REQUIRE(type == block_type);
                            }
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("contact types never share a padded block", "[contact_block_padding]")
{
    check_contact_type_block_layout<8>();
    check_contact_type_block_layout<12>();

    constexpr int IndexMax = std::numeric_limits<int>::max();
    const auto largest = make_contact_type_block_layout<8>(IndexMax - 7, 0, 0, 7);
    REQUIRE(largest.padded_total == IndexMax);
    REQUIRE_THROWS_AS(make_contact_type_block_layout<8>(IndexMax - 6, 0, 0, 0),
                      uipc::Exception);
}

TEST_CASE("contact block padding rejects IndexT overflow", "[contact_block_padding]")
{
    constexpr int IndexMax = std::numeric_limits<int>::max();

    const auto largest_layout = make_contact_type_block_layout<8>(IndexMax - 7, 0, 0, 7);
    REQUIRE(largest_layout.padded_total == IndexMax);

    REQUIRE_THROWS_AS(make_contact_type_block_layout<8>(IndexMax - 6, 0, 0, 0),
                      uipc::Exception);
}

TEST_CASE("contact continuous layout rejects aggregate IndexT overflow", "[contact_block_padding]")
{
    const auto ordinary =
        make_contact_type_contiguous_layout<int>(SizeT{1}, SizeT{2}, SizeT{3}, SizeT{4});
    REQUIRE(ordinary.pt_end == 1);
    REQUIRE(ordinary.ee_end == 3);
    REQUIRE(ordinary.pe_end == 6);
    REQUIRE(ordinary.pp_end == 10);

    constexpr SizeT IndexMax = static_cast<SizeT>(std::numeric_limits<int>::max());

    const auto largest = make_contact_type_contiguous_layout<int>(
        IndexMax - 3, SizeT{1}, SizeT{1}, SizeT{1});
    REQUIRE(largest.pt_end == std::numeric_limits<int>::max() - 3);
    REQUIRE(largest.ee_end == std::numeric_limits<int>::max() - 2);
    REQUIRE(largest.pe_end == std::numeric_limits<int>::max() - 1);
    REQUIRE(largest.pp_end == std::numeric_limits<int>::max());

    REQUIRE_THROWS_AS(make_contact_type_contiguous_layout<int>(
                          IndexMax - 2, SizeT{1}, SizeT{1}, SizeT{1}),
                      uipc::Exception);
}
