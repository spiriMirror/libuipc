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
#include <algorithm/fast_segmental_reduce.h>
#include <cub/warp/warp_reduce.cuh>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;

namespace
{
// The segmented reduction kernels as they were before one segment analysis
// per warp replaced the CUB HeadSegmentedReduce calls (one per value and one
// for the flags): the bitwise reference.
struct ReferenceFlags
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

template <int BlockSize, int WarpSize, typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
__global__ void reference_fast_segmental_reduce_scalar_kernel(cuda_tool::BufferView<T> out,
                                                              size_t in_size,
                                                              GetKeyOp get_key_op,
                                                              GetValueOp get_value_op,
                                                              ReduceOp op)
{
    using ValueT             = T;
    using Flags              = ReferenceFlags;
    constexpr int warp_size  = WarpSize;
    constexpr int warp_count = BlockSize / WarpSize;

    using WarpReduceInt = cub::WarpReduce<int, warp_size>;
    using WarpReduceT   = cub::WarpReduce<T, warp_size>;

    __shared__ union
    {
        typename WarpReduceInt::TempStorage index_storage[warp_count];
        typename WarpReduceT::TempStorage   t_storage[warp_count];
    };

    auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
    auto thread_id_in_block = threadIdx.x;
    auto warp_id            = thread_id_in_block / warp_size;
    auto lane_id            = thread_id_in_block & (warp_size - 1);

    int    prev_i = -1;
    int    next_i = -1;
    int    i      = -1;
    Flags  flags;
    ValueT value;
    flags.is_cross_warp = 0;

    if(global_thread_id > 0 && global_thread_id < in_size)
    {
        prev_i = get_key_op(global_thread_id - 1);
    }

    if(global_thread_id < in_size - 1)
    {
        next_i = get_key_op(global_thread_id + 1);
    }

    if(global_thread_id < in_size)
    {
        i              = get_key_op(global_thread_id);
        value          = get_value_op(global_thread_id);
        flags.is_valid = 1;
    }
    else
    {
        i                   = -1;
        value               = ValueT{0};
        flags.is_valid      = 0;
        flags.is_cross_warp = 0;
    }

    if(lane_id == 0)
    {
        flags.is_head       = 1;
        flags.is_cross_warp = reference_b2i(prev_i == i);
    }
    else
    {
        flags.is_head = reference_b2i(prev_i != i);

        if(lane_id == warp_size - 1)
        {
            flags.is_cross_warp = reference_b2i(next_i == i);
        }
    }

    flags.flags = WarpReduceInt(index_storage[warp_id])
                      .HeadSegmentedReduce(flags.flags, flags.is_head, op);

    value = WarpReduceT(t_storage[warp_id]).HeadSegmentedReduce(value, flags.is_head, op);


    if(flags.is_head && flags.is_valid)
    {
        if(flags.is_cross_warp)
        {
            auto& out_value = out(i);
            cuda_tool::atomic_add(&out_value, value);
        }
        else
        {
            out(i) = value;
        }
    }
}

template <int BlockSize, int WarpSize, typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
__global__ void reference_fast_segmental_reduce_matrix_kernel(
    cuda_tool::BufferView<Eigen::Matrix<T, M, N>> out,
    size_t                                        in_size,
    GetKeyOp                                      get_key_op,
    GetValueOp                                    get_value_op,
    ReduceOp                                      op)
{
    using Matrix             = Eigen::Matrix<T, M, N>;
    using Flags              = ReferenceFlags;
    constexpr int warp_size  = WarpSize;
    constexpr int warp_count = BlockSize / WarpSize;

    using WarpReduceInt = cub::WarpReduce<int, warp_size>;
    using WarpReduceT   = cub::WarpReduce<T, warp_size>;

    __shared__ union
    {
        typename WarpReduceInt::TempStorage index_storage[warp_count];
        typename WarpReduceT::TempStorage   t_storage[warp_count];
    };

    auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
    auto thread_id_in_block = threadIdx.x;
    auto warp_id            = thread_id_in_block / warp_size;
    auto lane_id            = thread_id_in_block & (warp_size - 1);

    int    prev_i = -1;
    int    next_i = -1;
    int    i      = -1;
    Flags  flags;
    Matrix value;
    flags.is_cross_warp = 0;

    if(global_thread_id > 0 && global_thread_id < in_size)
    {
        prev_i = get_key_op(global_thread_id - 1);
    }

    if(global_thread_id < in_size - 1)
    {
        next_i = get_key_op(global_thread_id + 1);
    }

    if(global_thread_id < in_size)
    {
        i              = get_key_op(global_thread_id);
        value          = get_value_op(global_thread_id);
        flags.is_valid = 1;
    }
    else
    {
        i = -1;
        value.setZero();
        flags.is_valid      = 0;
        flags.is_cross_warp = 0;
    }

    if(lane_id == 0)
    {
        flags.is_head       = 1;
        flags.is_cross_warp = reference_b2i(prev_i == i);
    }
    else
    {
        flags.is_head = reference_b2i(prev_i != i);

        if(lane_id == warp_size - 1)
        {
            flags.is_cross_warp = reference_b2i(next_i == i);
        }
    }

    flags.flags = WarpReduceInt(index_storage[warp_id])
                      .HeadSegmentedReduce(flags.flags, flags.is_head, op);

    for(int j = 0; j < M; j++)
    {
        for(int k = 0; k < N; k++)
        {
            value(j, k) = WarpReduceT(t_storage[warp_id])
                              .HeadSegmentedReduce(value(j, k), flags.is_head, op);
        }
    }

    if(flags.is_head && flags.is_valid)
    {
        if(flags.is_cross_warp)
        {
            auto& out_value = out(i);
            cuda_tool::eigen::atomic_add(out_value, value);
        }
        else
        {
            out(i) = value;
        }
    }
}

using Mat3 = Eigen::Matrix3d;

struct KeyOp
{
    cuda_tool::CBufferView<int> keys;
    __device__ int              operator()(int i) const { return keys(i); }
};

// gathers through a permutation, as MatrixConverter::convert_upper reads
// its sorted blocks
struct MatValueOp
{
    cuda_tool::CBufferView<Mat3> values;
    cuda_tool::CBufferView<int>  perm;
    __device__ Mat3 operator()(int i) const { return values(perm(i)); }
};

// a product, as the external articulation constraint's value op returns
struct ScalarProductOp
{
    cuda_tool::CBufferView<double> a;
    cuda_tool::CBufferView<double> b;
    cuda_tool::CBufferView<double> c;
    __device__ double operator()(int i) const { return a(i) * (b(i) - c(i)); }
};

struct IntValueOp
{
    cuda_tool::CBufferView<int> values;
    __device__ int              operator()(int i) const { return values(i); }
};

// Segment lengths: BCOO-like rows (a diagonal segment of 6..14 and 3..4
// off-diagonal ones of 2..6), single blocks, and segments spanning two and
// three or more warps.
std::vector<int> make_keys(std::mt19937_64& rng, int n)
{
    std::vector<int> keys;
    int              key = 0;
    while((int)keys.size() < n)
    {
        int len;
        switch(rng() % 10)
        {
            case 0:
                len = 1;
                break;
            case 1:
                len = 33 + int(rng() % 31);  // two or three warps
                break;
            case 2:
                len = 64 + int(rng() % 150);  // three to eight warps
                break;
            case 3:
                len = 2 + int(rng() % 5);
                break;
            default:
                len = 6 + int(rng() % 9);
                break;
        }
        for(int k = 0; k < len && (int)keys.size() < n; ++k)
            keys.push_back(key);
        ++key;
    }
    return keys;
}

// Pieces each segment is split into by the 32-lane warps, the trailing lanes
// past n joining the last segment.
std::vector<int> pieces_per_segment(const std::vector<int>& keys, int segments)
{
    std::vector<int> pieces(segments, 0);
    const int        n = (int)keys.size();
    for(int t = 0; t < n; ++t)
        if(t % 32 == 0 || keys[t - 1] != keys[t])
            pieces[keys[t]]++;
    return pieces;
}

// Random doubles over many magnitudes and both signs (signed zeros
// included); multiples of 1/16 below 2^10 where `exact`, so that any order
// of additions gives the same sum.
double make_value(std::mt19937_64& rng, bool exact)
{
    if(exact)
        return double(int(rng() % 4096) - 2048) / 16;
    switch(rng() % 16)
    {
        case 0:
            return 0.0;
        case 1:
            return -0.0;
        default: {
            std::uniform_real_distribution<double> u(-1.0, 1.0);
            return u(rng) * std::pow(10.0, int(rng() % 13) - 6);
        }
    }
}

template <typename T>
bool same_bits(const T& a, const T& b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

// The segment analysis with both forms of the warp maximum: on sm_80 and
// newer the default takes __reduce_max_sync, Shuffle = true takes the
// shuffles that the targets below sm_80 compile.
__global__ void warp_head_segments_kernel(cuda_tool::CBufferView<int> heads,
                                          cuda_tool::BufferView<int>  out)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    // heads holds whole warps, so this exit is uniform over every warp
    if(i >= static_cast<int>(heads.size()))
        return;
    const int  lane_id = threadIdx.x & 31;
    const int  is_head = lane_id == 0 ? 1 : heads(i);
    const auto native  = cuda_tool::warp_head_segments(is_head, lane_id);
    const auto shuffle = cuda_tool::warp_head_segments<true>(is_head, lane_id);
    out(4 * i + 0)     = native.last_lane;
    out(4 * i + 1)     = native.max_len;
    out(4 * i + 2)     = shuffle.last_lane;
    out(4 * i + 3)     = shuffle.max_len;
}
}  // namespace

TEST_CASE("fast segmental reduce keeps the bits of the CUB head-segmented reduction", "[segmental_reduce]")
{
    std::mt19937_64 rng(20261029);

    // in_size: within one warp, warp and block edges, a partial last warp,
    // a last block with wholly empty warps, and a large input
    for(int n : {1, 5, 31, 32, 33, 64, 127, 128, 129, 161, 4096, 4096 + 17, 50'000 + 3, 293'943})
    {
        INFO("in_size " << n);
        const std::vector<int> keys     = make_keys(rng, n);
        const int              segments = keys.back() + 1;
        const std::vector<int> pieces   = pieces_per_segment(keys, segments);
        // Segments of three or more pieces take three or more atomic
        // additions, whose order varies: their values are exact.
        std::vector<char> exact(n);
        for(int t = 0; t < n; ++t)
            exact[t] = pieces[keys[t]] >= 3;

        std::vector<int> perm(n);
        for(int t = 0; t < n; ++t)
            perm[t] = t;
        std::shuffle(perm.begin(), perm.end(), rng);
        std::vector<Mat3>   mats(n);
        std::vector<double> a(n), b(n), c(n);
        std::vector<int>    ints(n);
        // sorted position t reads mats[perm[t]]
        for(int t = 0; t < n; ++t)
            for(int k = 0; k < 9; ++k)
                mats[perm[t]].data()[k] = make_value(rng, exact[t]);
        for(int t = 0; t < n; ++t)
        {
            // exact products: a in 1/4 steps, b - c in 1/16 steps
            if(exact[t])
            {
                a[t] = double(int(rng() % 64) - 32) / 4;
                b[t] = double(int(rng() % 512) - 256) / 16;
                c[t] = double(int(rng() % 512) - 256) / 16;
            }
            else
            {
                a[t] = make_value(rng, false);
                b[t] = make_value(rng, false);
                c[t] = make_value(rng, false);
            }
            ints[t] = int(rng() % 2001) - 1000;
        }

        cuda_tool::DeviceBuffer<int>    d_keys(n), d_perm(n), d_ints(n);
        cuda_tool::DeviceBuffer<Mat3>   d_mats(n);
        cuda_tool::DeviceBuffer<double> d_a(n), d_b(n), d_c(n);
        d_keys.view().copy_from(keys.data());
        d_perm.view().copy_from(perm.data());
        d_ints.view().copy_from(ints.data());
        d_mats.view().copy_from(mats.data());
        d_a.view().copy_from(a.data());
        d_b.view().copy_from(b.data());
        d_c.view().copy_from(c.data());
        const KeyOp           key_op{d_keys.cview()};
        const MatValueOp      mat_op{d_mats.cview(), d_perm.cview()};
        const ScalarProductOp product_op{d_a.cview(), d_b.cview(), d_c.cview()};
        const IntValueOp      int_op{d_ints.cview()};

        // Outputs start as 1.0 (exact) where `prefill`, so a segment stored
        // where the reference adds (or the reverse) shows; one spare entry
        // past the last segment must stay untouched.
        for(bool prefill : {false, true})
        {
            INFO("prefill " << prefill);
            const Mat3 init_mat = prefill ? Mat3::Ones().eval() : Mat3::Zero().eval();
            const double        init = prefill ? 1.0 : 0.0;
            std::vector<Mat3>   h_mat_init(segments + 1, init_mat);
            std::vector<double> h_init(segments + 1, init);
            std::vector<int>    h_int_init(segments + 1, prefill ? 7 : 0);

            cuda_tool::DeviceBuffer<Mat3> ref_mat(segments + 1),
                new_mat(segments + 1), new_mat64(segments + 1);
            cuda_tool::DeviceBuffer<double> ref_sca(segments + 1), new_sca(segments + 1);
            cuda_tool::DeviceBuffer<int> ref_int(segments + 1), new_int(segments + 1);
            for(auto* buffer : {&ref_mat, &new_mat, &new_mat64})
                buffer->view().copy_from(h_mat_init.data());
            for(auto* buffer : {&ref_sca, &new_sca})
                buffer->view().copy_from(h_init.data());
            for(auto* buffer : {&ref_int, &new_int})
                buffer->view().copy_from(h_int_init.data());

            // The prefilled run compares only the store/add decision: a
            // segment adding onto 1.0 is exact only with exact values, so
            // there every value is exact.
            if(prefill)
            {
                std::vector<Mat3>   exact_mats(n);
                std::vector<double> ea(n), eb(n), ec(n);
                for(int t = 0; t < n; ++t)
                {
                    for(int k = 0; k < 9; ++k)
                        exact_mats[t].data()[k] = make_value(rng, true);
                    ea[t] = double(int(rng() % 64) - 32) / 4;
                    eb[t] = double(int(rng() % 512) - 256) / 16;
                    ec[t] = double(int(rng() % 512) - 256) / 16;
                }
                d_mats.view().copy_from(exact_mats.data());
                d_a.view().copy_from(ea.data());
                d_b.view().copy_from(eb.data());
                d_c.view().copy_from(ec.data());
            }

            const int blocks128 = (n + 127) / 128;
            const int blocks64  = (n + 63) / 64;
            reference_fast_segmental_reduce_matrix_kernel<128, 32, double, 3, 3>
                <<<blocks128, 128>>>(
                    ref_mat.view(0, segments), n, key_op, mat_op, ::cuda::std::plus<double>{});
            cuda_tool::FastSegmentalReduce<>().reduce_uncleared(
                n, new_mat.view(0, segments), key_op, mat_op);
            // a 64-thread block launches the same warps
            cuda_tool::FastSegmentalReduce<64, 32>().reduce_uncleared(
                n, new_mat64.view(0, segments), key_op, mat_op);
            reference_fast_segmental_reduce_scalar_kernel<128, 32, double>
                <<<blocks128, 128>>>(ref_sca.view(0, segments),
                                     n,
                                     key_op,
                                     product_op,
                                     ::cuda::std::plus<double>{});
            cuda_tool::FastSegmentalReduce<>().reduce_uncleared(
                n, new_sca.view(0, segments), key_op, product_op);
            reference_fast_segmental_reduce_scalar_kernel<64, 32, int><<<blocks64, 64>>>(
                ref_int.view(0, segments), n, key_op, int_op, ::cuda::std::plus<int>{});
            cuda_tool::FastSegmentalReduce<64, 32>().reduce_uncleared(
                n, new_int.view(0, segments), key_op, int_op);
            REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

            std::vector<Mat3> h_ref_mat(segments + 1), h_new_mat(segments + 1),
                h_new_mat64(segments + 1);
            std::vector<double> h_ref_sca(segments + 1), h_new_sca(segments + 1);
            std::vector<int> h_ref_int(segments + 1), h_new_int(segments + 1);
            ref_mat.view().copy_to(h_ref_mat.data());
            new_mat.view().copy_to(h_new_mat.data());
            new_mat64.view().copy_to(h_new_mat64.data());
            ref_sca.view().copy_to(h_ref_sca.data());
            new_sca.view().copy_to(h_new_sca.data());
            ref_int.view().copy_to(h_ref_int.data());
            new_int.view().copy_to(h_new_int.data());

            int mat_diff = 0, mat64_diff = 0, sca_diff = 0, int_diff = 0;
            for(int s = 0; s <= segments; ++s)
            {
                mat_diff += !same_bits(h_ref_mat[s], h_new_mat[s]);
                mat64_diff += !same_bits(h_ref_mat[s], h_new_mat64[s]);
                sca_diff += !same_bits(h_ref_sca[s], h_new_sca[s]);
                int_diff += h_ref_int[s] != h_new_int[s];
            }
            const int two_warps = (int)std::count(pieces.begin(), pieces.end(), 2);
            const int more_warps = (int)std::count_if(
                pieces.begin(), pieces.end(), [](int p) { return p >= 3; });
            INFO("segments " << segments << ", in two pieces " << two_warps
                             << ", in three or more " << more_warps);
            // the spare entry past the last segment
            CHECK(same_bits(h_new_mat[segments], init_mat));
            CHECK(same_bits(h_new_mat64[segments], init_mat));
            CHECK(same_bits(h_new_sca[segments], init));
            CHECK(h_new_int[segments] == (prefill ? 7 : 0));
            CHECK(mat_diff == 0);
            CHECK(mat64_diff == 0);
            CHECK(sca_diff == 0);
            CHECK(int_diff == 0);
            if(n > 4096)
            {
                CHECK(two_warps > 0);
                CHECK(more_warps > 0);
            }
        }
    }
}

TEST_CASE("warp segment analysis gives the same result with and without __reduce_max_sync",
          "[segmental_reduce]")
{
    // warps of heads: every lane, lane 0 only, lanes 0 and 31, then random
    constexpr int    warps = 67;
    std::vector<int> heads(warps * 32, 0);
    std::mt19937     rng(20261118);
    for(int lane = 0; lane < 32; ++lane)
        heads[lane] = 1;
    heads[2 * 32 + 31] = 1;
    for(int w = 3; w < warps; ++w)
        for(int lane = 0; lane < 32; ++lane)
            heads[w * 32 + lane] = (rng() % (1 + w % 7)) == 0;

    cuda_tool::DeviceBuffer<int> d_heads(heads.size()), d_out(4 * heads.size());
    d_heads.view().copy_from(heads.data());
    warp_head_segments_kernel<<<(warps * 32 + 127) / 128, 128>>>(d_heads.cview(),
                                                                 d_out.view());
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
    std::vector<int> out(4 * heads.size());
    d_out.view().copy_to(out.data());

    int wrong = 0;
    for(int w = 0; w < warps; ++w)
    {
        int last[32];
        int max_len = 0;
        for(int lane = 31, end = 31; lane >= 0; --lane)
        {
            last[lane] = end;
            if(lane == 0 || heads[w * 32 + lane])
            {
                max_len = std::max(max_len, end - lane + 1);
                end     = lane - 1;
            }
        }
        for(int lane = 0; lane < 32; ++lane)
        {
            const int* o = &out[4 * (w * 32 + lane)];
            wrong += o[0] != last[lane] || o[1] != max_len || o[2] != last[lane]
                     || o[3] != max_len;
        }
    }
    CHECK(wrong == 0);
}
