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
#include <affine_body/abd_diag_inverse.h>

#include <cstring>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
// The previous one-thread-per-body assembly kernel.
__global__ void serial_abd_diag_inverse_kernel(cuda_tool::CBufferView<Matrix12x12> diag_hessian,
                                               cuda_tool::BufferView<Matrix12x12> diag_inv)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= (int)diag_hessian.size())
        return;
    diag_inv(i) = cuda_tool::eigen::inverse(diag_hessian(i));
}

Matrix12x12 random_matrix(std::mt19937_64& rng, Float scale)
{
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    Matrix12x12                           m;
    for(int c = 0; c < 12; ++c)
        for(int r = 0; r < 12; ++r)
            m(r, c) = scale * u(rng);
    return m;
}

Matrix12x12 random_spd(std::mt19937_64& rng, Float scale)
{
    const Matrix12x12 b = random_matrix(rng, 1.0);
    return scale * (b * b.transpose() + 0.05 * Matrix12x12::Identity());
}

// Kinetic (sum of point masses through the affine Jacobian [I, x0 I, x1 I,
// x2 I]) plus a stiff SPD shape term on the 9 affine DoFs, like the ABD
// diagonal Hessian of a non-fixed body.
Matrix12x12 abd_like(std::mt19937_64& rng)
{
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    std::uniform_real_distribution<Float> pos(0.0, 1.0);
    const Float                           half_extent = 0.05 + 0.5 * pos(rng);
    Matrix12x12                           h           = Matrix12x12::Zero();
    for(int k = 0; k < 8; ++k)
    {
        Eigen::Matrix<Float, 3, 12> J = Eigen::Matrix<Float, 3, 12>::Zero();
        Vector3 x{half_extent * u(rng), half_extent * u(rng), half_extent * u(rng)};
        J.block<3, 3>(0, 0) = Matrix3x3::Identity();
        for(int a = 0; a < 3; ++a)
            J.block<3, 3>(0, 3 + 3 * a) = x(a) * Matrix3x3::Identity();
        h += (0.1 + pos(rng)) * J.transpose() * J;
    }
    Eigen::Matrix<Float, 9, 9> s = Eigen::Matrix<Float, 9, 9>::Zero();
    for(int k = 0; k < 9; ++k)
    {
        Eigen::Matrix<Float, 9, 1> v;
        for(int a = 0; a < 9; ++a)
            v(a) = u(rng);
        s += v * v.transpose();
    }
    h.block<9, 9>(3, 3) += 1e8 * (pos(rng) + 0.01) * s;
    return h;
}

// Sparse non-symmetric matrices whose exact zeros and negative pivots
// exercise the signed-zero results of the elimination.
Matrix12x12 sparse_general(std::mt19937_64& rng)
{
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    Matrix12x12                           m = Matrix12x12::Zero();
    for(int c = 0; c < 12; ++c)
        for(int r = 0; r < 12; ++r)
            if(u(rng) > 0.4)
                m(r, c) = u(rng);
    for(int r = 0; r < 12; ++r)
        m(r, r) = (u(rng) < 0 ? -1.0 : 1.0) * (2.0 + u(rng));
    return m;
}
}  // namespace

TEST_CASE("ABD diagonal warp inverse is bitwise identical to the serial Gauss elimination",
          "[abd][abd_diag_inverse]")
{
    std::mt19937_64          rng(20261027);
    std::vector<Matrix12x12> blocks;
    int                      pivot_repairs = 0;

    for(int k = 0; k < 64; ++k)  // fixed bodies
        blocks.push_back(Matrix12x12::Identity());
    for(Float scale : {1e-8, 1e-3, 1.0, 1e3, 1e9})
        for(int k = 0; k < 768; ++k)
            blocks.push_back(random_spd(rng, scale));
    for(int k = 0; k < 2048; ++k)
        blocks.push_back(abd_like(rng));
    for(int k = 0; k < 1024; ++k)
        blocks.push_back(random_matrix(rng, 1.0));
    for(int k = 0; k < 1024; ++k)
        blocks.push_back(sparse_general(rng));

    // Pivot repair on the input: m[0][0] == 0 (repaired with row 1) and a
    // zero diagonal (repairs whenever an elimination leaves a pivot < eps).
    for(int k = 0; k < 256; ++k)
    {
        Matrix12x12 m = random_matrix(rng, 1.0);
        m(0, 0)       = 0.0;
        blocks.push_back(m);
        ++pivot_repairs;

        Matrix12x12 z = random_matrix(rng, 1.0);
        z.diagonal().setZero();
        blocks.push_back(z);
        ++pivot_repairs;
    }
    // Pivots below eps repaired with a later row: |m[0][0]|, |m[1][0]| < eps.
    for(int k = 0; k < 256; ++k)
    {
        Matrix12x12 m = random_matrix(rng, 1.0);
        m(0, 0)       = (k % 2 ? -1.0 : 1.0) * 1e-17;
        m(1, 0)       = 3e-18;
        blocks.push_back(m);
        ++pivot_repairs;
    }
    // A pivot that becomes 2^-53 < eps during elimination: rows 0 and 1 agree
    // in their first two columns up to one ulp of 0.5.
    for(int k = 0; k < 256; ++k)
    {
        Matrix12x12 m = random_matrix(rng, 1.0);
        m(0, 0)       = 1.0;
        m(0, 1)       = 1.0;
        m(1, 0)       = 0.5;
        m(1, 1)       = 0.5 + std::ldexp(1.0, -53);
        blocks.push_back(m);
        ++pivot_repairs;
    }
    // Nearly singular SPD blocks: rank 11 plus 1e-14 * I.
    for(int k = 0; k < 256; ++k)
    {
        Matrix12x12 c = random_matrix(rng, 1.0);
        c.col(k % 12).setZero();
        blocks.push_back(c * c.transpose() + 1e-14 * Matrix12x12::Identity());
    }

    const SizeT n = blocks.size();
    INFO("blocks " << n << ", constructed pivot repairs " << pivot_repairs);

    cuda_tool::DeviceBuffer<Matrix12x12> d_hessian(n);
    cuda_tool::DeviceBuffer<Matrix12x12> d_serial(n);
    cuda_tool::DeviceBuffer<Matrix12x12> d_warp(n);
    d_hessian.view().copy_from(blocks.data());

    // Different garbage in the two outputs: an unwritten block cannot match.
    std::vector<Matrix12x12> serial(n), warp(n);
    std::memset(serial.data(), 0xab, n * sizeof(Matrix12x12));
    std::memset(warp.data(), 0xcd, n * sizeof(Matrix12x12));
    d_serial.view().copy_from(serial.data());
    d_warp.view().copy_from(warp.data());

    serial_abd_diag_inverse_kernel<<<(int)((n + 63) / 64), 64>>>(d_hessian.cview(),
                                                                 d_serial.view());
    launch_abd_diag_inverse(d_hessian.cview(), d_warp.view(), nullptr);
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

    d_serial.view().copy_to(serial.data());
    d_warp.view().copy_to(warp.data());

    SizeT first_mismatch = n;
    SizeT mismatches     = 0;
    for(SizeT i = 0; i < n; ++i)
    {
        if(std::memcmp(&serial[i], &warp[i], sizeof(Matrix12x12)) != 0)
        {
            if(first_mismatch == n)
                first_mismatch = i;
            ++mismatches;
        }
    }
    INFO("first mismatching block " << first_mismatch);
    CHECK(mismatches == 0);
    CHECK(std::memcmp(serial.data(), warp.data(), n * sizeof(Matrix12x12)) == 0);

    // The comparison is meaningful: the inverses are real inverses.
    for(SizeT i = 0; i < 64; ++i)
        CHECK(warp[i] == Matrix12x12::Identity());
    const Matrix12x12 residual = blocks[64] * warp[64] - Matrix12x12::Identity();
    CHECK(residual.cwiseAbs().maxCoeff() < 1e-6);
}

TEST_CASE("ABD diagonal warp inverse stores the identity for a block without a usable pivot",
          "[abd][abd_diag_inverse]")
{
    // The serial routine returns an uninitialized matrix for these blocks.
    std::mt19937_64          rng(7);
    std::vector<Matrix12x12> blocks{Matrix12x12::Zero()};
    for(int c = 0; c < 12; ++c)
    {
        Matrix12x12 m = random_matrix(rng, 1.0);
        m.col(c).setZero();
        blocks.push_back(m);
    }

    const SizeT                          n = blocks.size();
    cuda_tool::DeviceBuffer<Matrix12x12> d_hessian(n);
    cuda_tool::DeviceBuffer<Matrix12x12> d_warp(n);
    d_hessian.view().copy_from(blocks.data());
    std::vector<Matrix12x12> warp(n);
    std::memset(warp.data(), 0xcd, n * sizeof(Matrix12x12));
    d_warp.view().copy_from(warp.data());

    launch_abd_diag_inverse(d_hessian.cview(), d_warp.view(), nullptr);
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
    d_warp.view().copy_to(warp.data());

    for(SizeT i = 0; i < n; ++i)
    {
        INFO("block " << i);
        CHECK(warp[i] == Matrix12x12::Identity());
    }
}
