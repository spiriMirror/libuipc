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
#include <affine_body/abd_hessian_blocks.h>
#include <utils/matrix_unpacker.h>

#include <cstring>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
UIPC_GENERIC void zero_out_lower(Matrix12x12& H)
{
    for(IndexT jj = 0; jj < 4; ++jj)
        for(IndexT ii = jj + 1; ii < 4; ++ii)
            H.block<3, 3>(ii * 3, jj * 3).setZero();
}

// The former one-thread-per-12x12-block kernels, kept as the reference.
__global__ void reference_kinetic_shape_kernel(
    cuda_tool::TripletMatrixView<Float, 3> dst,
    cuda_tool::CBufferView<IndexT>         is_fixed,
    cuda_tool::CBufferView<IndexT>         is_external_kinetic,
    cuda_tool::CBufferView<Matrix12x12>    shape_hessian,
    cuda_tool::CBufferView<Matrix12x12>    kinetic_hessian,
    cuda_tool::BufferView<Matrix12x12>     diag_hessian,
    int                                    n)
{
    int I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= n)
        return;
    TripletMatrixUnpacker MA{dst};
    Matrix12x12           H12x12;
    if(is_fixed(I))
    {
        H12x12.setIdentity();
    }
    else
    {
        H12x12 = shape_hessian(I);
        if(!is_external_kinetic(I))
            H12x12 += kinetic_hessian(I);
    }
    diag_hessian(I) = H12x12;
    zero_out_lower(H12x12);
    MA.block<4, 4>(I * 4 * 4).write(I * 4, I * 4, H12x12);
}

__global__ void reference_reporters_kernel(cuda_tool::TripletMatrixView<Float, 3> dst,
                                           cuda_tool::CTripletMatrixView<Float, 12, 12> src,
                                           cuda_tool::BufferView<Matrix12x12> diag_hessian,
                                           cuda_tool::CBufferView<IndexT> is_fixed,
                                           int n)
{
    int I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= n)
        return;
    TripletMatrixUnpacker MU{dst};
    Matrix12x12           H12x12;
    auto&& [body_i, body_j, Value] = src(I);
    H12x12                         = Value;

    bool has_fixed = (is_fixed(body_i) || is_fixed(body_j));
    if(body_i == body_j && !has_fixed)
        cuda_tool::eigen::atomic_add(diag_hessian(body_i), H12x12);

    if(has_fixed)
        H12x12.setZero();
    else if(body_i == body_j)
        zero_out_lower(H12x12);
    else if(body_i > body_j)
        H12x12.setZero();

    MU.block<4, 4>(I * 4 * 4).write(body_i * 4, body_j * 4, H12x12);
}

Matrix12x12 random_matrix(std::mt19937_64& rng)
{
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    Matrix12x12                           m;
    for(int c = 0; c < 12; ++c)
        for(int r = 0; r < 12; ++r)
            m(r, c) = u(rng) * std::pow(10.0, 6.0 * u(rng));
    return m;
}

struct Triplets
{
    std::vector<int>       rows;
    std::vector<int>       cols;
    std::vector<Matrix3x3> values;
};

Triplets download(cuda_tool::DeviceTripletMatrix<Float, 3>& m)
{
    Triplets t;
    t.rows.resize(m.triplet_count());
    t.cols.resize(m.triplet_count());
    t.values.resize(m.triplet_count());
    m.row_indices().copy_to(t.rows.data());
    m.col_indices().copy_to(t.cols.data());
    m.values().copy_to(t.values.data());
    return t;
}

bool same(const Triplets& a, const Triplets& b)
{
    return a.rows == b.rows && a.cols == b.cols
           && std::memcmp(a.values.data(), b.values.data(), a.values.size() * sizeof(Matrix3x3))
                  == 0;
}

void fill_garbage(cuda_tool::DeviceTripletMatrix<Float, 3>& m, int byte)
{
    std::vector<int>       idx(m.triplet_count());
    std::vector<Matrix3x3> val(m.triplet_count());
    std::memset(idx.data(), byte, idx.size() * sizeof(int));
    std::memset(val.data(), byte, val.size() * sizeof(Matrix3x3));
    m.row_indices().copy_from(idx.data());
    m.col_indices().copy_from(idx.data());
    m.values().copy_from(val.data());
}
}  // namespace

TEST_CASE("ABD Hessian gathers per 3x3 block are bitwise identical to the per-12x12-block kernels",
          "[abd][abd_hessian_blocks]")
{
    std::mt19937_64 rng(20261027);
    const int       bodies = 77;  // not a multiple of the 4 bodies per block

    std::vector<IndexT>      is_fixed(bodies), is_external(bodies);
    std::vector<Matrix12x12> shape(bodies), kinetic(bodies);
    for(int b = 0; b < bodies; ++b)
    {
        is_fixed[b]    = b % 7 == 3;
        is_external[b] = b % 5 == 1;
        shape[b]       = random_matrix(rng);
        kinetic[b]     = random_matrix(rng);
    }

    // Reporter blocks: off-diagonal pairs in both orders, pairs with fixed
    // bodies, and at most one diagonal block per body (so that the atomic
    // additions of the reference and the new kernel are deterministic).
    std::vector<int>         rep_i, rep_j;
    std::vector<Matrix12x12> rep_value;
    for(int b = 0; b < bodies; ++b)
    {
        if(b % 2 == 0)
        {
            rep_i.push_back(b);
            rep_j.push_back(b);
            rep_value.push_back(random_matrix(rng));
        }
        int other = (b * 31 + 7) % bodies;
        if(other != b)
        {
            rep_i.push_back(b);
            rep_j.push_back(other);
            rep_value.push_back(random_matrix(rng));
        }
    }
    const int reporters = (int)rep_i.size();

    cuda_tool::DeviceBuffer<IndexT>      d_fixed(bodies), d_external(bodies);
    cuda_tool::DeviceBuffer<Matrix12x12> d_shape(bodies), d_kinetic(bodies);
    d_fixed.view().copy_from(is_fixed.data());
    d_external.view().copy_from(is_external.data());
    d_shape.view().copy_from(shape.data());
    d_kinetic.view().copy_from(kinetic.data());

    cuda_tool::DeviceTripletMatrix<Float, 12, 12> d_reporters;
    d_reporters.resize(bodies, bodies, reporters);
    d_reporters.row_indices().copy_from(rep_i.data());
    d_reporters.col_indices().copy_from(rep_j.data());
    d_reporters.values().copy_from(rep_value.data());

    const int triplets = (bodies + reporters) * 16;
    const auto run = [&](bool reference, Triplets& out, std::vector<Matrix12x12>& diag)
    {
        cuda_tool::DeviceTripletMatrix<Float, 3> d_out;
        d_out.resize(bodies * 4, bodies * 4, triplets);
        fill_garbage(d_out, reference ? 0xab : 0xcd);
        cuda_tool::DeviceBuffer<Matrix12x12> d_diag(bodies);
        std::vector<Matrix12x12>             garbage(bodies);
        std::memset(garbage.data(), reference ? 0xab : 0xcd, bodies * sizeof(Matrix12x12));
        d_diag.view().copy_from(garbage.data());

        auto body_view     = d_out.view().subview(0, bodies * 16);
        auto reporter_view = d_out.view().subview(bodies * 16, reporters * 16);
        if(reference)
        {
            reference_kinetic_shape_kernel<<<(bodies + 63) / 64, 64>>>(
                body_view,
                d_fixed.cview(),
                d_external.cview(),
                d_shape.cview(),
                d_kinetic.cview(),
                d_diag.view(),
                bodies);
            reference_reporters_kernel<<<(reporters + 63) / 64, 64>>>(
                reporter_view, d_reporters.cview(), d_diag.view(), d_fixed.cview(), reporters);
        }
        else
        {
            launch_abd_kinetic_shape_hessian_blocks(body_view,
                                                    d_fixed.cview(),
                                                    d_external.cview(),
                                                    d_shape.cview(),
                                                    d_kinetic.cview(),
                                                    d_diag.view(),
                                                    nullptr);
            launch_abd_reporter_hessian_blocks(
                reporter_view, d_reporters.cview(), d_diag.view(), d_fixed.cview(), nullptr);
        }
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
        out = download(d_out);
        diag.resize(bodies);
        d_diag.view().copy_to(diag.data());
    };

    Triplets                 ref_out, new_out;
    std::vector<Matrix12x12> ref_diag, new_diag;
    run(true, ref_out, ref_diag);
    run(false, new_out, new_diag);

    CHECK(same(ref_out, new_out));
    CHECK(std::memcmp(ref_diag.data(), new_diag.data(), bodies * sizeof(Matrix12x12)) == 0);

    // The comparison is meaningful: a free body's stored block is its sum.
    CHECK(new_diag[0] == shape[0] + kinetic[0] + rep_value[0]);
}
