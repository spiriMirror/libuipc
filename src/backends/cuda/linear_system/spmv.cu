/*
 * NVIDIA contributions: fused sparse-matrix/vector multiplication and
 * dot-product launch paths; the device triplet-count guard that skips warps
 * past the live triplets, the unit-scale specialization, one segment analysis
 * per warp with a shared diagonal/off-diagonal dot path, and 128-thread
 * blocks.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

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

#include <linear_system/spmv.h>
#include <algorithm/fast_segmental_reduce.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <cub/warp/warp_reduce.cuh>
#include <cub/warp/warp_scan.cuh>
#include <cub/util_math.cuh>
#include <cuda_device/bit_operation.h>
#include <cuda_device/builtin.h>
#include <Eigen/Sparse>

namespace uipc::backend::cuda
{
__host__ __device__ constexpr int b2i(bool b)
{
    return b ? 1 : 0;
}

struct Flags
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

    __host__ __device__ void b2i()
    {
        is_head       = is_head ? 1 : 0;
        is_cross_warp = is_cross_warp ? 1 : 0;
        is_valid      = is_valid ? 1 : 0;
    }
};

namespace
{
    // Four warps per block, each still reducing its own 32 triplets: the
    // grid covers the BCOO capacity, so a call with nothing to do (the
    // pipelined no-op slot) schedules a quarter of the blocks; the dot and y
    // atomics keep their addends and only arrive in another order.
    constexpr int RbkSymSpmvDotBlockDim = 128;
    static_assert(RbkSymSpmvDotBlockDim >= 32);
    static_assert(RbkSymSpmvDotBlockDim <= 1024);
    static_assert(RbkSymSpmvDotBlockDim % 32 == 0);

    // y = b * y, shared by sym_spmv / rbk_spmv / rbk_sym_spmv / rbk_sym_spmv_dot
    __global__ void Spmv_scale_y_kernel(Float b, cuda_tool::DenseVectorView<Float> y, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        y(i) = b * y(i);
    }

    __global__ void Spmv_sym_spmv_kernel(Float                                a,
                                         cuda_tool::CBCOOMatrixView<Float, 3> A,
                                         cuda_tool::CDenseVectorView<Float>   x,
                                         cuda_tool::DenseVectorView<Float>    y,
                                         int                                  n)
    {
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        if(index >= n)
            return;

        constexpr int N = 3;
        using T         = Float;

        auto&& [i, j, block] = A(index);

        if(i == j)  // diagonal block
        {
            auto seg_x = x.segment<N>(j * N);

            Eigen::Vector<T, N> vec_x  = seg_x.as_eigen();
            auto                result = a * block * vec_x;

            auto seg_y = y.segment<N>(i * N);
            seg_y.atomic_add(result.eval());
        }
        else  // off-diagonal block
        {
            // ij-th block
            {
                auto seg_x = x.segment<N>(j * N);

                Eigen::Vector<T, N> vec_x  = seg_x.as_eigen();
                auto                result = a * block * vec_x;

                auto seg_y = y.segment<N>(i * N);
                seg_y.atomic_add(result.eval());
            }

            // ji-th block
            {
                auto seg_x = x.segment<N>(i * N);

                Eigen::Vector<T, N> vec_x  = seg_x.as_eigen();
                auto                result = a * block.transpose() * vec_x;

                auto seg_y = y.segment<N>(j * N);
                seg_y.atomic_add(result.eval());
            }
        }
    }

    __global__ void Spmv_rbk_spmv_kernel(Float                                a,
                                         cuda_tool::CBCOOMatrixView<Float, 3> A,
                                         cuda_tool::CDenseVectorView<Float>   x,
                                         cuda_tool::DenseVectorView<Float>    y)
    {
        constexpr int          warp_size = 32;
        constexpr unsigned int warp_mask = ~0u;
        constexpr int          block_dim = 128;
        constexpr int          N         = 3;

        using WarpReduceInt   = cub::WarpReduce<int, warp_size>;
        using WarpReduceFloat = cub::WarpReduce<Float, warp_size>;
        using WarpScanInt     = cub::WarpScan<int>;

        auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
        auto thread_id_in_block = threadIdx.x;
        auto warp_id            = thread_id_in_block / warp_size;
        auto lane_id            = thread_id_in_block & (warp_size - 1);

        __shared__ union
        {
            typename WarpReduceInt::TempStorage temp_storage_int[block_dim / warp_size];
            typename WarpReduceFloat::TempStorage temp_storage_float[block_dim / warp_size];
        };

        int prev_i = -1;
        int next_i = -1;
        int i      = -1;

        Flags   flags;
        Vector3 vec;
        flags.is_cross_warp = 0;


        if(global_thread_id > 0 && global_thread_id < A.triplet_count())
        {
            auto prev_triplet = A(global_thread_id - 1);
            prev_i            = prev_triplet.row_index;
        }

        if(global_thread_id < A.triplet_count() - 1)
        {
            auto next_triplet = A(global_thread_id + 1);
            next_i            = next_triplet.row_index;
        }

        if(global_thread_id < A.triplet_count())
        {
            auto Triplet = A(global_thread_id);
            i            = Triplet.row_index;
            auto j       = Triplet.col_index;

            vec = Triplet.value * x.segment<N>(j * N).as_eigen();

            flags.is_valid = 1;
        }
        else
        {
            i = -1;
            vec.setZero();
            flags.is_valid      = 0;
            flags.is_cross_warp = 0;
        }

        if(lane_id == 0)
        {
            flags.is_head = 1;
            // if this thread is the first thread in the warp
            // check if the previous triplet is in the same row
            // if so, this row crosses the warp boundary, we need use atomic add
            flags.is_cross_warp = b2i(prev_i == i);
        }
        else
        {
            flags.is_head = b2i(prev_i != i);  // must be 1 or 0, or the result is undefined

            if(lane_id == warp_size - 1)
            {
                // if this thread is the last thread in the warp
                // check if the next triplet is in the same row
                // if so, this row crosses the warp boundary, we need use atomic add
                flags.is_cross_warp = b2i(next_i == i);
            }
        }

        flags.flags =
            WarpReduceInt(temp_storage_int[warp_id])
                .HeadSegmentedReduce(flags.flags,
                                     flags.is_head,
                                     [](uint32_t a, uint32_t b) { return a + b; });

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


        // cub::WARP_SYNC(warp_mask);

        flags.is_head = b2i(flags.is_head && flags.is_valid);

        flags.b2i();
        int is_head_mask = detail::bit_operation::WARP_BALLOT(flags.is_head, warp_mask);
        uint32_t offset = __fns(is_head_mask, 0, lane_id + 1);

        int valid_bit = (offset != ~0u);
        int shuffle_mask = detail::bit_operation::WARP_BALLOT(valid_bit, warp_mask);

        i           = cub::ShuffleIndex<32>(i, offset, shuffle_mask);
        flags.flags = cub::ShuffleIndex<32>(flags.flags, offset, shuffle_mask);
        vec.x()     = cub::ShuffleIndex<32>(vec.x(), offset, shuffle_mask);
        vec.y()     = cub::ShuffleIndex<32>(vec.y(), offset, shuffle_mask);
        vec.z()     = cub::ShuffleIndex<32>(vec.z(), offset, shuffle_mask);

        if(valid_bit && flags.is_head && flags.is_valid)
        {
            auto seg_y  = y.segment<N>(i * N);
            auto result = a * vec;

            if(flags.is_cross_warp)
            {
                seg_y.atomic_add(result.eval());
            }
            else
            {
                seg_y.as_eigen() += result.eval();
            }
        }
    }

    __global__ void Spmv_rbk_sym_spmv_kernel(Float a,
                                             cuda_tool::CBCOOMatrixView<Float, 3> A,
                                             cuda_tool::CDenseVectorView<Float> x,
                                             cuda_tool::DenseVectorView<Float> y,
                                             int n)
    {
        constexpr int warp_size = 32;
        constexpr int block_dim = 256;
        constexpr int N         = 3;

        for(int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < n;
            idx += gridDim.x * blockDim.x)
        {
            using WarpReduceInt   = cub::WarpReduce<int, warp_size>;
            using WarpReduceFloat = cub::WarpReduce<Float, warp_size>;
            using WarpScanInt     = cub::WarpScan<int>;

            auto global_thread_id   = idx;
            auto thread_id_in_block = threadIdx.x;
            auto warp_id            = thread_id_in_block / warp_size;
            auto lane_id            = thread_id_in_block & (warp_size - 1);

            __shared__ union
            {
                typename WarpReduceInt::TempStorage temp_storage_int[block_dim / warp_size];
                typename WarpReduceFloat::TempStorage temp_storage_float[block_dim / warp_size];
            };

            int     prev_i = -1;
            int     i      = -1;
            Flags   flags;
            Vector3 vec;

            // In symmtric version, we don't need to check the cross warp
            flags.is_cross_warp = 0;

            // set the previous row index
            if(global_thread_id > 0)
            {
                auto prev_triplet = A(global_thread_id - 1);
                prev_i            = prev_triplet.row_index;
            }

            {
                auto Triplet = A(global_thread_id);
                i            = Triplet.row_index;
                auto j       = Triplet.col_index;

                vec = Triplet.value * x.segment<N>(j * N).as_eigen();

                flags.is_valid = 1;

                if(i != j)  // process lower triangle
                {
                    Vector3 vec_ = a * Triplet.value.transpose()
                                   * x.segment<N>(i * N).as_eigen();

                    y.segment<N>(j * N).atomic_add(vec_);
                }
            }

            if(lane_id == 0)
            {
                flags.is_head = 1;
            }
            else
            {
                flags.is_head = b2i(prev_i != i);  // must be 1 or 0, or the result is undefined
            }


            // ----------------------------------- warp reduce ----------------------------------------------
            vec.x() =
                WarpReduceFloat(temp_storage_float[warp_id])
                    .HeadSegmentedReduce(vec.x(),
                                         flags.is_head,
                                         [](Float a, Float b) { return a + b; });

            vec.y() =
                WarpReduceFloat(temp_storage_float[warp_id])
                    .HeadSegmentedReduce(vec.y(),
                                         flags.is_head,
                                         [](Float a, Float b) { return a + b; });

            vec.z() =
                WarpReduceFloat(temp_storage_float[warp_id])
                    .HeadSegmentedReduce(vec.z(),
                                         flags.is_head,
                                         [](Float a, Float b) { return a + b; });
            // ----------------------------------- warp reduce -----------------------------------------------


            if(flags.is_head)
            {
                auto seg_y  = y.segment<N>(i * N);
                auto result = a * vec;

                // Must use atomic add!
                // Because the same row may be processed by different warps
                seg_y.atomic_add(result.eval());
            }
        }
    }

    // Head-segmented warp sums of the three components of v, each with
    // exactly the additions of CUB's WarpReduce<Float, 32>::HeadSegmentedReduce
    // (shuffle variant): at offsets 1, 2, 4, 8 and 16 a lane adds the value of
    // lane + offset when that lane is in its segment. No lane adds anything at
    // an offset of at least the warp's longest segment, so those steps are
    // skipped (warp-uniformly; rows of the BCOO upper triangle are a few
    // blocks long, so usually the last one or two of the five). One loop
    // serves the three components, so their shuffle-add chains interleave.
    __device__ __forceinline__ void spmv_segmented_sum(Vector3& v,
                                                       const cuda_tool::WarpHeadSegments& seg,
                                                       int lane_id)
    {
#pragma unroll
        for(int offset = 1; offset < 32; offset <<= 1)
        {
            if(offset >= seg.max_len)
                break;
            const Float ox = __shfl_down_sync(0xffffffffu, v.x(), offset);
            const Float oy = __shfl_down_sync(0xffffffffu, v.y(), offset);
            const Float oz = __shfl_down_sync(0xffffffffu, v.z(), offset);
            if(offset + lane_id <= seg.last_lane)
            {
                v.x() = v.x() + ox;
                v.y() = v.y() + oy;
                v.z() = v.z() + oz;
            }
        }
    }

    // UnitScale: the caller passes a == 1 (the fused PCG always does). The
    // kernel then skips every multiplication by a (13 of the about 55 FP64
    // instructions of an off-diagonal triplet; GB20x runs FP64 at 1:64 and
    // this kernel is FP64-bound). Multiplying by 1 is exact, so the results
    // are bitwise those of the general kernel with a == 1.
    template <bool Pipelined, bool UnitScale>
    __global__ void Spmv_rbk_sym_spmv_dot_kernel(Float a,
                                                 cuda_tool::CBCOOMatrixView<Float, 3> A,
                                                 cuda_tool::CDenseVectorView<Float> x,
                                                 cuda_tool::DenseVectorView<Float> y,
                                                 cuda_tool::Dense<Float> d_dot,
                                                 cuda_tool::CDense<IndexT> d_triplet_count,
                                                 Float*        next_y,
                                                 int           next_y_size,
                                                 Float*        next_dot,
                                                 const IndexT* converged)
    {
        auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
        auto thread_id_in_block = threadIdx.x;

        // count lives on device: a graph capturing this kernel then stays
        // valid when the matrix nnz changes within the reserved capacity
        // (read together with the converged flag)
        const int triplet_count = (int)(*d_triplet_count);
        if constexpr(Pipelined)
        {
            // Every thread observes the same stream-ordered scalar. Returning
            // here is therefore uniform across the block and cannot strand a
            // later collective.
            if(*converged != 0)
                return;

            for(int i = global_thread_id; i < next_y_size;
                i += gridDim.x * blockDim.x)
                next_y[i] = Float{0};
            if(global_thread_id == 0)
                *next_dot = Float{0};
        }

        constexpr int warp_size = 32;

        // The grid covers the capacity, which can be several times the live
        // count. A warp wholly past the count writes nothing, so it leaves
        // here (warp-uniform) instead of running the FP64 warp reductions.
        if((int)(global_thread_id & ~(warp_size - 1u)) >= triplet_count)
            return;
        constexpr int block_dim = RbkSymSpmvDotBlockDim;
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

        int     prev_i = -1;
        int     i      = -1;
        Flags   flags;
        Vector3 vec;
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

            // One dot path for diagonal and off-diagonal blocks, so a warp
            // holding both issues it once: x_i is x_j on the diagonal, and
            // the factor (1 or 2, times a) is exact, so every lane gets the
            // bits of its former branch (x_j.dot(vec) times a, or
            // 2 * a * x_i.dot(vec)).
            Eigen::Vector<T, N> x_i = x.segment<N>(i * N).as_eigen();
            if constexpr(UnitScale)
                dot_local = (i == j ? T{1} : T{2}) * x_i.dot(vec);
            else
                dot_local = (i == j ? a : T{2} * a) * x_i.dot(vec);

            if(i != j)
            {
                Vector3 vec_;
                if constexpr(UnitScale)
                    vec_ = vals[global_thread_id].transpose() * x_i;
                else
                    vec_ = a * vals[global_thread_id].transpose() * x_i;
                y.segment<N>(j * N).atomic_add(vec_);
            }

            if(lane_id == 0)
                flags.is_head = 1;
            else
                flags.is_head = b2i(prev_i != i);
        }

        const cuda_tool::WarpHeadSegments segments =
            cuda_tool::warp_head_segments(flags.is_head, lane_id);
        spmv_segmented_sum(vec, segments, lane_id);

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

void Spmv::sym_spmv(Float                                a,
                    cuda_tool::CBCOOMatrixView<Float, 3> A,
                    cuda_tool::CDenseVectorView<Float>   x,
                    Float                                b,
                    cuda_tool::DenseVectorView<Float>    y)
{
    if(b != 0)
    {
        int n = y.size();
        if(n > 0)
        {
            Spmv_scale_y_kernel<<<cuda_tool::best_grid_dim(n, Spmv_scale_y_kernel), cuda_tool::best_block_dim(Spmv_scale_y_kernel), 0, nullptr>>>(
                b, y, n);
        }
    }
    else
    {
        cuda_tool::BufferLaunch().fill<Float>(y.buffer_view(), 0);
    }

    int triplet_count = A.triplet_count();
    if(triplet_count > 0)
    {
        Spmv_sym_spmv_kernel<<<cuda_tool::best_grid_dim(triplet_count, Spmv_sym_spmv_kernel), cuda_tool::best_block_dim(Spmv_sym_spmv_kernel), 0, nullptr>>>(
            a, A, x, y, triplet_count);
    }
}

void Spmv::rbk_spmv(Float                                a,
                    cuda_tool::CBCOOMatrixView<Float, 3> A,
                    cuda_tool::CDenseVectorView<Float>   x,
                    Float                                b,
                    cuda_tool::DenseVectorView<Float>    y)
{
    if(b != 0)
    {
        int n = y.size();
        if(n > 0)
        {
            Spmv_scale_y_kernel<<<cuda_tool::best_grid_dim(n, Spmv_scale_y_kernel), cuda_tool::best_block_dim(Spmv_scale_y_kernel), 0, nullptr>>>(
                b, y, n);
        }
    }
    else
    {
        cuda_tool::BufferLaunch().fill<Float>(y.buffer_view(), 0);
    }

    constexpr int block_dim   = 128;
    int           block_count = (A.triplet_count() + block_dim - 1) / block_dim;

    if(block_count > 0)
    {
        Spmv_rbk_spmv_kernel<<<block_count, block_dim, 0, nullptr>>>(a, A, x, y);
    }
}

void Spmv::rbk_sym_spmv(Float                                a,
                        cuda_tool::CBCOOMatrixView<Float, 3> A,
                        cuda_tool::CDenseVectorView<Float>   x,
                        Float                                b,
                        cuda_tool::DenseVectorView<Float>    y)

{
    if(b != 0)
    {
        int n = y.size();
        if(n > 0)
        {
            Spmv_scale_y_kernel<<<cuda_tool::best_grid_dim(n, Spmv_scale_y_kernel), cuda_tool::best_block_dim(Spmv_scale_y_kernel), 0, nullptr>>>(
                b, y, n);
        }
    }
    else
    {
        cuda_tool::BufferLaunch().fill<Float>(y.buffer_view(), 0);
    }

    constexpr int block_dim   = 256;
    int           block_count = (A.triplet_count() + block_dim - 1) / block_dim;
    int           triplet_count = A.triplet_count();

    if(triplet_count > 0)
    {
        Spmv_rbk_sym_spmv_kernel<<<block_count, block_dim, 0, nullptr>>>(a, A, x, y, triplet_count);
    }
}

void Spmv::rbk_sym_spmv_dot(Float                                a,
                            cuda_tool::CBCOOMatrixView<Float, 3> A,
                            cuda_tool::CDenseVectorView<Float>   x,
                            Float                                b,
                            cuda_tool::DenseVectorView<Float>    y,
                            cuda_tool::VarView<Float>            d_dot,
                            cuda_tool::CDense<IndexT> d_triplet_count,
                            SizeT                     triplet_capacity,
                            cudaStream_t              stream)
{
    if(b != 0)
    {
        int n = y.size();
        if(n > 0)
        {
            Spmv_scale_y_kernel<<<cuda_tool::best_grid_dim(n, Spmv_scale_y_kernel), cuda_tool::best_block_dim(Spmv_scale_y_kernel), 0, stream>>>(
                b, y, n);
        }
    }
    else
    {
        cuda_tool::BufferLaunch(stream).fill<Float>(y.buffer_view(), 0);
    }

    cudaMemsetAsync(d_dot.data(), 0, sizeof(Float), stream);

    // grid covers the reserved capacity: blocks beyond the current
    // (device-side) count exit through the is_valid guard with zero work,
    // so the launch shape need not change when the count does
    constexpr int block_dim = RbkSymSpmvDotBlockDim;
    int block_count = (int)((triplet_capacity + block_dim - 1) / block_dim);

    if(block_count > 0)
    {
        // a == 1 (every fused PCG call) takes the kernel without the scale
        // multiplications; it computes bitwise the same y and dot.
        auto kernel = a == Float{1} ? Spmv_rbk_sym_spmv_dot_kernel<false, true> :
                                      Spmv_rbk_sym_spmv_dot_kernel<false, false>;
        kernel<<<block_count, block_dim, 0, stream>>>(
            a, A, x, y, d_dot.viewer(), d_triplet_count, nullptr, 0, nullptr, nullptr);
    }
}

void Spmv::rbk_sym_spmv_dot_pipelined(Float                                a,
                                      cuda_tool::CBCOOMatrixView<Float, 3> A,
                                      cuda_tool::CDenseVectorView<Float>   x,
                                      cuda_tool::DenseVectorView<Float>    y,
                                      cuda_tool::VarView<Float>         d_dot,
                                      cuda_tool::DenseVectorView<Float> next_y,
                                      cuda_tool::VarView<Float>   next_dot,
                                      cuda_tool::CVarView<IndexT> converged,
                                      cuda_tool::CDense<IndexT> d_triplet_count,
                                      SizeT        triplet_capacity,
                                      cudaStream_t stream)
{
    constexpr int block_dim = RbkSymSpmvDotBlockDim;
    int block_count = (int)((triplet_capacity + block_dim - 1) / block_dim);

    // Keep the clear contract valid for an empty-capacity matrix as well.
    if(block_count == 0 && next_y.size() != 0)
        block_count = 1;

    if(block_count > 0)
    {
        // a == 1 (every fused PCG call): no scale multiplications, same bits
        auto kernel = a == Float{1} ? Spmv_rbk_sym_spmv_dot_kernel<true, true> :
                                      Spmv_rbk_sym_spmv_dot_kernel<true, false>;
        kernel<<<block_count, block_dim, 0, stream>>>(a,
                                                      A,
                                                      x,
                                                      y,
                                                      d_dot.viewer(),
                                                      d_triplet_count,
                                                      next_y.data(),
                                                      (int)next_y.size(),
                                                      next_dot.data(),
                                                      converged.data());
    }
}

void Spmv::cpu_sym_spmv(Float                                a,
                        cuda_tool::CBCOOMatrixView<Float, 3> A_view,
                        cuda_tool::CDenseVectorView<Float>   x,
                        Float                                b,
                        cuda_tool::DenseVectorView<Float>    y)
{
    using BlockMatrix      = Matrix3x3;
    constexpr int BlockDim = 3;

    // Get matrix dimensions
    int total_block_rows  = A_view.total_rows();
    int total_block_cols  = A_view.total_cols();
    int total_scalar_rows = total_block_rows * BlockDim;
    int total_scalar_cols = total_block_cols * BlockDim;

    // Copy triplet data from device to host
    std::vector<int>         row_indices_host(A_view.triplet_count());
    std::vector<int>         col_indices_host(A_view.triplet_count());
    std::vector<BlockMatrix> values_host(A_view.triplet_count());

    A_view.row_indices().copy_to(row_indices_host.data());
    A_view.col_indices().copy_to(col_indices_host.data());
    A_view.values().copy_to(values_host.data());

    // Build Eigen SparseMatrix from symmetric block-sparse representation
    std::vector<Eigen::Triplet<Float>> triplets;
    triplets.reserve(A_view.triplet_count() * BlockDim * BlockDim * 2);  // Reserve for symmetric expansion

    for(size_t t = 0; t < A_view.triplet_count(); ++t)
    {
        int                block_i = row_indices_host[t];
        int                block_j = col_indices_host[t];
        const BlockMatrix& block   = values_host[t];

        // Convert block indices to scalar indices
        int scalar_i_base = block_i * BlockDim;
        int scalar_j_base = block_j * BlockDim;

        // Add all entries from the block (upper triangular part)
        for(int bi = 0; bi < BlockDim; ++bi)
        {
            for(int bj = 0; bj < BlockDim; ++bj)
            {
                Float value    = block(bi, bj);
                int   scalar_i = scalar_i_base + bi;
                int   scalar_j = scalar_j_base + bj;

                triplets.emplace_back(scalar_i, scalar_j, value);
            }
        }

        // Since matrix is symmetric at block level, also add transpose block (lower triangular part)
        if(block_i != block_j)
        {
            BlockMatrix block_transpose = block.transpose();
            for(int bi = 0; bi < BlockDim; ++bi)
            {
                for(int bj = 0; bj < BlockDim; ++bj)
                {
                    Float value  = block_transpose(bi, bj);
                    int scalar_i = scalar_j_base + bi;  // Swapped base indices
                    int scalar_j = scalar_i_base + bj;

                    triplets.emplace_back(scalar_i, scalar_j, value);
                }
            }
        }
    }

    // Build Eigen SparseMatrix
    Eigen::SparseMatrix<Float> A_sparse(total_scalar_rows, total_scalar_cols);
    A_sparse.setFromTriplets(triplets.begin(), triplets.end());
    A_sparse.makeCompressed();

    // Copy vectors from device to host
    Eigen::VectorX<Float> x_host(x.size());
    Eigen::VectorX<Float> y_host(y.size());

    x.buffer_view().copy_to(x_host.data());
    y.buffer_view().copy_to(y_host.data());

    // Compute y = a * A * x + b * y using Eigen sparse matrix-vector multiplication
    y_host = a * A_sparse * x_host + b * y_host;

    // Copy result back to device
    y.buffer_view().copy_from(y_host.data());
}
}  // namespace uipc::backend::cuda
