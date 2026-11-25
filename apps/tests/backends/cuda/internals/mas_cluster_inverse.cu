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
#include <cuda_tool/cuda_tool.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
constexpr int BankSize = MASPreconditionerEngine::BANKSIZE;
constexpr int MatDim   = BankSize * 3;
using ClusterSym       = MASPreconditionerEngine::ClusterMatrixSym;
using ClusterSymF      = MASPreconditionerEngine::ClusterMatrixSymF;
using Dense48          = Eigen::Matrix<Float, MatDim, MatDim>;

UIPC_GENERIC int sym_index(int row, int col)
{
    return BankSize * row - row * (row + 1) / 2 + col;
}

// The former shared-memory inversion kernel, kept as the reference.
__global__ void reference_invert_cluster_matrices_kernel(cuda_tool::BufferView<ClusterSymF> cluster_inv,
                                                         cuda_tool::CBufferView<ClusterSym> cluster_hess,
                                                         int total_threads)
{
    using namespace cuda_tool;

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= total_threads)
        return;

    constexpr int MAT_DIM = BankSize * 3;  // 48

    int mat_id       = idx / MAT_DIM;
    int col          = idx % MAT_DIM;
    int block_mat_id = threadIdx.x / MAT_DIM;

    __shared__ double s_mat_raw[32 / BankSize][MAT_DIM][MAT_DIM];
    __shared__ double s_col_raw[32 / BankSize][MAT_DIM];
    auto s_mat = make_dense_2d(&s_mat_raw[block_mat_id][0][0], MAT_DIM, MAT_DIM);
    auto s_col = make_dense_1d(&s_col_raw[block_mat_id][0], MAT_DIM);

    for(int row = 0; row < MAT_DIM; row++)
    {
        int node_row = row / 3;
        int node_col = col / 3;
        if(node_col >= node_row)
        {
            int si          = sym_index(node_row, node_col);
            s_mat(row, col) = cluster_hess(mat_id).M[si](row % 3, col % 3);
        }
        else
        {
            int si          = sym_index(node_col, node_row);
            s_mat(row, col) = cluster_hess(mat_id).M[si](col % 3, row % 3);
        }
        if(row == col && s_mat(row, col) == 0.0)
            s_mat(row, col) = 1.0;
    }

    for(int pivot = 0; pivot < MAT_DIM; pivot++)
    {
        __syncthreads();
        double pivot_val = s_mat(pivot, pivot);
        s_col(col)       = s_mat(col, pivot);
        __syncthreads();

        s_mat(col == pivot ? col : col, pivot) = (col == pivot) ? 1.0 : 0.0;

        __syncthreads();
        s_mat(pivot, col) /= pivot_val;
        __syncthreads();

        for(int row = 0; row < MAT_DIM; row++)
        {
            if(row != pivot)
            {
                double factor = -s_col(row);
                s_mat(row, col) += factor * s_mat(pivot, col);
            }
        }
    }
    __syncthreads();

    if(col % 3 < 2)
        s_mat(col + 1, col) = s_mat(col, col + 1);
    else
        s_mat(col, col - 2) = s_mat(col - 2, col);
    __syncthreads();

    for(int row = 0; row < MAT_DIM; row++)
    {
        int node_row = row / 3;
        int node_col = col / 3;
        if(node_col >= node_row)
        {
            int si = sym_index(node_row, node_col);
            cluster_inv(mat_id).M[si](row % 3, col % 3) =
                static_cast<float>(s_mat(row, col));
        }
    }
}

// The 24-thread symmetric-sweep kernel the 96-thread one replaced, kept as
// the bitwise reference: thread t of a cluster owns column t (rows 0..t) and
// column 47 - t (row 48 - s in slot s > t), 4 clusters per 96-thread block.
__global__ void reference_sweep_invert_cluster_matrices_kernel(
    cuda_tool::BufferView<ClusterSymF> cluster_inv,
    cuda_tool::CBufferView<ClusterSym> cluster_hess,
    int                                total_threads)
{
    constexpr int MAT_DIM  = BankSize * 3;  // 48
    constexpr int HALF     = MAT_DIM / 2;   // threads per cluster
    constexpr int SLOTS    = MAT_DIM + 1;   // upper-triangle values per thread
    constexpr int CLUSTERS = 4;             // per 96-thread block

    const int  idx    = blockIdx.x * blockDim.x + threadIdx.x;
    const bool active = idx < total_threads;
    const int  mat_id = idx / HALF;
    const int  t      = idx % HALF;
    const int  b      = threadIdx.x / HALF;
    const int  p      = t;
    const int  q      = MAT_DIM - 1 - t;

    __shared__ double s_c[CLUSTERS][MAT_DIM];
    __shared__ double s_r[CLUSTERS][MAT_DIM];

    double a[SLOTS];
#pragma unroll
    for(int s = 0; s < SLOTS; ++s)
    {
        const bool first = s <= p;
        const int  row   = first ? s : MAT_DIM - s;
        const int  col   = first ? p : q;
        double     v     = 1.0;
        if(active)
            v = cluster_hess(mat_id).M[sym_index(row / 3, col / 3)](row % 3, col % 3);
        if(row == col && v == 0.0)
            v = 1.0;
        a[s] = v;
    }

    for(int k = 0; k < MAT_DIM; ++k)
    {
        const bool p_is_k = p == k;
        const bool q_is_k = q == k;
#pragma unroll
        for(int s = 0; s < SLOTS; ++s)
        {
            if(s <= p)
            {
                if(p_is_k)
                    s_c[b][s] = a[s];
                else if(s == k)
                    s_c[b][p] = a[s];
            }
            else
            {
                if(q_is_k)
                    s_c[b][MAT_DIM - s] = a[s];
                else if(MAT_DIM - s == k)
                    s_c[b][q] = a[s];
            }
        }
        __syncthreads();
        const double inv_d = 1.0 / s_c[b][k];
        const double r_p   = s_c[b][p] * inv_d;
        const double r_q   = s_c[b][q] * inv_d;
        s_r[b][p]          = r_p;
        s_r[b][q]          = r_q;
        __syncthreads();
#pragma unroll
        for(int s = 0; s < SLOTS; ++s)
        {
            const bool   first = s <= p;
            const int    row   = first ? s : MAT_DIM - s;
            const bool   col_k = first ? p_is_k : q_is_k;
            const double r_col = first ? r_p : r_q;
            const double c_row = first ? s_c[b][s] : s_c[b][MAT_DIM - s];
            const double r_row = first ? s_r[b][s] : s_r[b][MAT_DIM - s];
            if(row == k && col_k)
                a[s] = -inv_d;
            else if(col_k)
                a[s] = r_row;
            else if(row == k)
                a[s] = r_col;
            else
                a[s] = fma(-c_row, r_col, a[s]);
        }
        __syncthreads();
    }

    if(!active)
        return;
#pragma unroll
    for(int s = 0; s < SLOTS; ++s)
    {
        const bool  first = s <= p;
        const int   row   = first ? s : MAT_DIM - s;
        const int   col   = first ? p : q;
        const int   si    = sym_index(row / 3, col / 3);
        const float v     = static_cast<float>(-a[s]);
        cluster_inv(mat_id).M[si](row % 3, col % 3) = v;
        if(row / 3 == col / 3 && row < col)
            cluster_inv(mat_id).M[si](col % 3, row % 3) = v;
    }
}

ClusterSym to_cluster(const Dense48& a)
{
    ClusterSym c;
    for(int nr = 0; nr < BankSize; ++nr)
        for(int nc = nr; nc < BankSize; ++nc)
            c.M[sym_index(nr, nc)] = a.block<3, 3>(nr * 3, nc * 3);
    return c;
}

Dense48 random_spd(std::mt19937_64& rng, Float scale)
{
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    Dense48                               b;
    for(int c = 0; c < MatDim; ++c)
        for(int r = 0; r < MatDim; ++r)
            b(r, c) = u(rng);
    return scale * (b * b.transpose() + 0.05 * Dense48::Identity());
}

// Lumped masses plus spring stiffness along random node pairs: the shape of a
// MAS cluster of an FEM Hessian (dominant diagonal 3x3 blocks, coupled
// off-diagonal blocks, a few nodes unconnected).
Dense48 fem_like(std::mt19937_64& rng)
{
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    std::uniform_real_distribution<Float> pos(0.0, 1.0);
    Dense48                               k = Dense48::Zero();
    for(int n = 0; n < BankSize; ++n)
        k.block<3, 3>(n * 3, n * 3) += (1e-3 + 1e-2 * pos(rng)) * Matrix3x3::Identity();
    for(int e = 0; e < 40; ++e)
    {
        int i = int(pos(rng) * BankSize) % BankSize;
        int j = int(pos(rng) * BankSize) % BankSize;
        if(i == j)
            continue;
        Vector3 d{u(rng), u(rng), u(rng)};
        d.normalize();
        Matrix3x3 s = (1e3 + 1e5 * pos(rng))
                      * (d * d.transpose() + 1e-3 * Matrix3x3::Identity());
        k.block<3, 3>(i * 3, i * 3) += s;
        k.block<3, 3>(j * 3, j * 3) += s;
        k.block<3, 3>(i * 3, j * 3) -= s;
        k.block<3, 3>(j * 3, i * 3) -= s;
    }
    return k;
}

// dense symmetric 48x48 from the upper-triangle block storage
template <typename Cluster>
Dense48 to_dense(const Cluster& c)
{
    Dense48 d;
    for(int nr = 0; nr < BankSize; ++nr)
        for(int nc = 0; nc < BankSize; ++nc)
        {
            if(nc >= nr)
                d.block<3, 3>(nr * 3, nc * 3) =
                    c.M[sym_index(nr, nc)].template cast<Float>();
            else
                d.block<3, 3>(nr * 3, nc * 3) =
                    c.M[sym_index(nc, nr)].template cast<Float>().transpose();
        }
    return d;
}
}  // namespace

TEST_CASE("MAS cluster inversion by symmetric sweeps matches the Gauss-Jordan inverse",
          "[mas][mas_cluster_inverse]")
{
    std::mt19937_64         rng(20261027);
    std::vector<ClusterSym> clusters;

    clusters.push_back(to_cluster(Dense48::Identity()));
    clusters.push_back(to_cluster(Dense48::Zero()));  // all pivots repaired to 1
    for(Float scale : {1e-6, 1.0, 1e6})
        for(int k = 0; k < 64; ++k)
            clusters.push_back(to_cluster(random_spd(rng, scale)));
    for(int k = 0; k < 256; ++k)
        clusters.push_back(to_cluster(fem_like(rng)));
    // Padding nodes: the trailing nodes of a partially filled cluster are zero.
    for(int k = 0; k < 64; ++k)
    {
        Dense48 a    = fem_like(rng);
        int     used = 3 * (1 + k % (BankSize - 1));
        a.block(used, 0, MatDim - used, MatDim).setZero();
        a.block(0, used, MatDim, MatDim - used).setZero();
        clusters.push_back(to_cluster(a));
    }
    // a cluster count that leaves idle threads in the last block
    clusters.push_back(to_cluster(fem_like(rng)));
    if(clusters.size() % 4 == 0)
        clusters.push_back(to_cluster(fem_like(rng)));

    const int n = (int)clusters.size();
    INFO("clusters " << n);

    cuda_tool::DeviceBuffer<ClusterSym>  d_hess(n);
    cuda_tool::DeviceBuffer<ClusterSymF> d_reference(n);
    cuda_tool::DeviceBuffer<ClusterSymF> d_sweep(n);
    d_hess.view().copy_from(clusters.data());

    // Garbage in the outputs: an unwritten entry cannot pass.
    std::vector<ClusterSymF> reference(n), sweep(n);
    std::memset(reference.data(), 0xab, n * sizeof(ClusterSymF));
    std::memset(sweep.data(), 0xab, n * sizeof(ClusterSymF));
    d_reference.view().copy_from(reference.data());
    d_sweep.view().copy_from(sweep.data());

    const int total_threads = n * MatDim;
    reference_invert_cluster_matrices_kernel<<<(total_threads + 95) / 96, 96>>>(
        d_reference.view(), d_hess.cview(), total_threads);
    MASPreconditionerEngine::invert_cluster_matrices(d_hess.cview(), d_sweep.view(), n, nullptr);
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

    d_reference.view().copy_to(reference.data());
    d_sweep.view().copy_to(sweep.data());

    // Both are the float inverse of the same matrix computed in double by a
    // different elimination order: they agree to rounding, scaled by the
    // conditioning of the cluster.
    int    worst_cluster = -1;
    double worst         = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const Dense48 r     = to_dense(reference[i]);
        const Dense48 x     = to_dense(sweep[i]);
        const double  scale = r.cwiseAbs().maxCoeff();
        REQUIRE(std::isfinite(x.cwiseAbs().maxCoeff()));
        const double diff = (x - r).cwiseAbs().maxCoeff() / scale;
        if(diff > worst)
        {
            worst         = diff;
            worst_cluster = i;
        }
    }
    INFO("worst cluster " << worst_cluster << " relative difference " << worst);
    CHECK(worst < 1e-5);

    // exact where no rounding happens
    CHECK(to_dense(sweep[0]) == Dense48::Identity());
    CHECK(to_dense(sweep[1]) == Dense48::Identity());

    // the results are inverses (well-conditioned SPD clusters of scale 1)
    for(int k = 0; k < 64; ++k)
    {
        const Dense48 h        = to_dense(clusters[2 + 64 + k]);
        const Dense48 x        = to_dense(sweep[2 + 64 + k]);
        const Dense48 residual = h * x - Dense48::Identity();
        CHECK(residual.cwiseAbs().maxCoeff() < 1e-3);
        const Dense48 exact = h.inverse();
        CHECK((x - exact).cwiseAbs().maxCoeff() < 1e-6 * exact.cwiseAbs().maxCoeff());
    }
}

TEST_CASE("MAS cluster inversion by 96-thread blocks keeps the bits of the 24-thread sweep",
          "[mas][mas_cluster_inverse]")
{
    std::mt19937_64         rng(20261028);
    std::vector<ClusterSym> clusters;

    clusters.push_back(to_cluster(Dense48::Identity()));
    clusters.push_back(to_cluster(Dense48::Zero()));
    // SPD clusters over the range of scales an IPC Hessian spans
    for(Float scale : {1e-8, 1e-4, 1.0, 1e4, 1e9})
        for(int k = 0; k < 400; ++k)
            clusters.push_back(to_cluster(random_spd(rng, scale)));
    for(int k = 0; k < 6000; ++k)
        clusters.push_back(to_cluster(std::pow(10.0, k % 9 - 4) * fem_like(rng)));
    // padding nodes: the trailing nodes of a partially filled cluster are zero
    for(int k = 0; k < 1500; ++k)
    {
        Dense48 a    = fem_like(rng);
        int     used = 3 * (1 + k % (BankSize - 1));
        a.block(used, 0, MatDim - used, MatDim).setZero();
        a.block(0, used, MatDim, MatDim - used).setZero();
        clusters.push_back(to_cluster(a));
    }
    // a zero row and column anywhere: its diagonal entry is repaired to 1
    for(int k = 0; k < 500; ++k)
    {
        Dense48 a = fem_like(rng);
        a.row(k % MatDim).setZero();
        a.col(k % MatDim).setZero();
        clusters.push_back(to_cluster(a));
    }

    const int n = (int)clusters.size();
    INFO("clusters " << n);

    cuda_tool::DeviceBuffer<ClusterSym>  d_hess(n);
    cuda_tool::DeviceBuffer<ClusterSymF> d_reference(n);
    cuda_tool::DeviceBuffer<ClusterSymF> d_inverse(n);
    cuda_tool::DeviceBuffer<ClusterSymF> d_skipped(n);
    d_hess.view().copy_from(clusters.data());

    // Distinct garbage in every output: an unwritten entry cannot pass.
    std::vector<ClusterSymF> reference(n), inverse(n), skipped(n), untouched(1);
    std::memset(reference.data(), 0xab, n * sizeof(ClusterSymF));
    std::memset(inverse.data(), 0xcd, n * sizeof(ClusterSymF));
    std::memset(skipped.data(), 0xef, n * sizeof(ClusterSymF));
    std::memset(untouched.data(), 0xef, sizeof(ClusterSymF));
    d_reference.view().copy_from(reference.data());
    d_inverse.view().copy_from(inverse.data());
    d_skipped.view().copy_from(skipped.data());

    const int total_threads = n * MatDim / 2;
    reference_sweep_invert_cluster_matrices_kernel<<<(total_threads + 95) / 96, 96>>>(
        d_reference.view(), d_hess.cview(), total_threads);
    MASPreconditionerEngine::invert_cluster_matrices(d_hess.cview(), d_inverse.view(), n, nullptr);
    // skip a middle range, like the empty level-0 banks of the MAS hierarchy
    const int skip_begin = n / 3;
    const int skip_count = n / 5;
    MASPreconditionerEngine::invert_cluster_matrices(
        d_hess.cview(), d_skipped.view(), n, nullptr, skip_begin, skip_count);
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

    d_reference.view().copy_to(reference.data());
    d_inverse.view().copy_to(inverse.data());
    d_skipped.view().copy_to(skipped.data());

    int mismatches = 0, skip_mismatches = 0, skip_written = 0;
    for(int i = 0; i < n; ++i)
    {
        mismatches += std::memcmp(&reference[i], &inverse[i], sizeof(ClusterSymF)) != 0;
        if(i >= skip_begin && i < skip_begin + skip_count)
            skip_written +=
                std::memcmp(&untouched[0], &skipped[i], sizeof(ClusterSymF)) != 0;
        else
            skip_mismatches +=
                std::memcmp(&reference[i], &skipped[i], sizeof(ClusterSymF)) != 0;
    }
    CHECK(mismatches == 0);
    CHECK(skip_mismatches == 0);
    CHECK(skip_written == 0);
    CHECK(to_dense(inverse[0]) == Dense48::Identity());
    CHECK(to_dense(inverse[1]) == Dense48::Identity());
}
