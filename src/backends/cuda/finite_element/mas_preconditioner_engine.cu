#include <finite_element/mas_preconditioner_engine.h>
#include <cuda_tool/cuda_tool.h>
#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include <uipc/common/log.h>
#include <cuda_runtime.h>
#include <cub/warp/warp_reduce.cuh>
#include <fmt/format.h>
#include <uipc/common/json.h>
#include <fstream>
#include <vector>

namespace uipc::backend::cuda
{
// ============================================================================
// Local constants & type aliases
// ============================================================================
static constexpr int BANKSIZE = MASPreconditionerEngine::BANKSIZE;
static constexpr int DEFAULT_BLOCKSIZE = MASPreconditionerEngine::DEFAULT_BLOCKSIZE;
static constexpr int DEFAULT_WARPNUM = MASPreconditionerEngine::DEFAULT_WARPNUM;
static constexpr int SYM_BLOCK_COUNT = MASPreconditionerEngine::SYM_BLOCK_COUNT;

using ClusterMatrixSym  = MASPreconditionerEngine::ClusterMatrixSym;
using ClusterMatrixSymF = MASPreconditionerEngine::ClusterMatrixSymF;
using LevelTable        = MASPreconditionerEngine::LevelTable;
using Int2              = MASPreconditionerEngine::Int2;

// Device helper: bitmask with bits [0, lane_id) set
UIPC_GENERIC unsigned int lanemask_lt(int lane_id)
{
    return (1U << lane_id) - 1;
}

// Symmetric upper-triangle index for a BANKSIZE x BANKSIZE block
UIPC_GENERIC int sym_index(int row, int col)
{
    return BANKSIZE * row - row * (row + 1) / 2 + col;
}

// Round n up to the nearest multiple of BANKSIZE
constexpr int bank_align(int n)
{
    return (n + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
}

// ============================================================================
// Named kernels (replacements for the former lambda kernel launches)
// ============================================================================
namespace
{
    __global__ void MASPreconditionerEngine_build_connect_mask_L0_kernel(
        cuda_tool::CBufferView<unsigned int> neighbor_start,
        cuda_tool::BufferView<unsigned int>  neighbor_num,
        cuda_tool::BufferView<unsigned int>  neighbor_list,
        cuda_tool::BufferView<unsigned int>  fine_connect_mask,
        cuda_tool::CBufferView<int>          part_to_real,
        cuda_tool::CBufferView<int>          real_to_part,
        int                                  n)
    {
        using namespace cuda_tool;

        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if(tid >= n)
            return;

        int bank_id = tid / BANKSIZE;
        int lane_id = tid % BANKSIZE;
        int idx     = part_to_real(tid);

        if(idx < 0)
            return;

        int          num_nbr     = neighbor_num(idx);
        unsigned int connect_msk = (1U << lane_id);
        int          nk          = 0;
        int          start_id    = neighbor_start(idx);

        for(int i = 0; i < num_nbr; i++)
        {
            int nbr_id      = neighbor_list(start_id + i);
            int nbr_part_id = real_to_part(nbr_id);
            if(nbr_part_id < 0)
            {
                continue;
            }
            int nbr_bank_id = nbr_part_id / BANKSIZE;
            if(bank_id == nbr_bank_id)
            {
                unsigned int nbr_lane = nbr_part_id % BANKSIZE;
                connect_msk |= (1U << nbr_lane);
            }
            else
            {
                neighbor_list(start_id + nk) = nbr_id;
                nk++;
            }
        }
        neighbor_num(idx)      = nk;
        fine_connect_mask(idx) = connect_msk;
    }

    __global__ void MASPreconditionerEngine_prepare_prefix_sum_L0_kernel(
        cuda_tool::BufferView<unsigned int> fine_connect_mask,
        cuda_tool::BufferView<int>          prefix_orig,
        cuda_tool::CBufferView<int>         part_to_real,
        int                                 N)
    {
        using namespace cuda_tool;

        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if(tid >= N)
            return;

        int warp_id       = tid / BANKSIZE;
        int local_warp_id = threadIdx.x / BANKSIZE;
        int lane_id       = tid % BANKSIZE;
        int idx           = part_to_real(tid);

        __shared__ unsigned int cache_mask_s[DEFAULT_BLOCKSIZE];
        __shared__ int          prefix_sum_s[DEFAULT_WARPNUM];
        auto cache_mask = make_dense_1d(cache_mask_s);
        auto prefix_sum = make_dense_1d(prefix_sum_s);

        if(idx >= 0)
        {
            unsigned int connect_msk = fine_connect_mask(idx);
            if(lane_id == 0)
                prefix_sum(local_warp_id) = 0;
            cache_mask(threadIdx.x) = connect_msk;
            unsigned int visited    = (1U << lane_id);

            while(connect_msk != ~0U)
            {
                unsigned int todo = visited ^ connect_msk;
                if(!todo)
                    break;
                unsigned int next_visit = __ffs(todo) - 1;
                visited |= (1U << next_visit);
                connect_msk |= cache_mask(next_visit + local_warp_id * BANKSIZE);
            }

            fine_connect_mask(idx) = connect_msk;

            unsigned int elected_prefix =
                __popc(connect_msk & lanemask_lt(lane_id));
            if(elected_prefix == 0)
                atomicAdd(&prefix_sum(local_warp_id), 1);
            if(lane_id == 0)
                prefix_orig(warp_id) = prefix_sum(local_warp_id);
        }
    }

    __global__ void MASPreconditionerEngine_build_level1_kernel(
        cuda_tool::BufferView<Int2>        level_size,
        cuda_tool::BufferView<int>           coarse_table,
        cuda_tool::BufferView<int>           going_next_L0,
        cuda_tool::CBufferView<unsigned int> fine_connect_mask,
        cuda_tool::CBufferView<int>          prefix_sum_orig,
        cuda_tool::CBufferView<int>          prefix_orig,
        cuda_tool::CBufferView<int>          part_to_real,
        int                                  N,
        int                                  level1_begin)
    {
        using namespace cuda_tool;

        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if(tid >= N)
            return;

        int warp_id       = tid / BANKSIZE;
        int local_warp_id = threadIdx.x / BANKSIZE;
        int lane_id       = tid % BANKSIZE;

        __shared__ unsigned int elected_mask_s[BANKSIZE];
        __shared__ unsigned int lane_prefix_s[BANKSIZE * BANKSIZE];
        auto elected_mask = make_dense_1d(elected_mask_s);
        auto lane_prefix  = make_dense_1d(lane_prefix_s);

        if(lane_id == 0)
            elected_mask(local_warp_id) = 0;

        if(tid == N - 1)
        {
            level_size(1).x = prefix_sum_orig(warp_id) + prefix_orig(warp_id);
            level_size(1).y = level1_begin;
        }

        int idx = part_to_real(tid);
        if(idx >= 0)
        {
            unsigned int conn_msk = fine_connect_mask(idx);
            unsigned int elected_prefix = __popc(conn_msk & lanemask_lt(lane_id));
            if(elected_prefix == 0)
                atomicOr(&elected_mask(local_warp_id), (1U << lane_id));

            lane_prefix(threadIdx.x) =
                __popc(elected_mask(local_warp_id) & lanemask_lt(lane_id));
            lane_prefix(threadIdx.x) += prefix_sum_orig(warp_id);

            unsigned int elected_lane = __ffs(conn_msk) - 1;
            unsigned int the_lane_prefix =
                lane_prefix(elected_lane + BANKSIZE * local_warp_id);

            coarse_table(idx)   = the_lane_prefix;
            going_next_L0(idx) = the_lane_prefix + level1_begin;
        }
    }

    __global__ void MASPreconditionerEngine_build_connect_mask_Lx_kernel(
        cuda_tool::CBufferView<unsigned int> neighbor_start,
        cuda_tool::BufferView<unsigned int>  neighbor_num,
        cuda_tool::BufferView<unsigned int>  neighbor_list,
        cuda_tool::CBufferView<int>          coarse_table,
        cuda_tool::BufferView<unsigned int>  next_connect_mask,
        cuda_tool::CBufferView<unsigned int> fine_connect_mask,
        cuda_tool::CBufferView<int>          part_to_real,
        int                                  level,
        int                                  vert_num,
        int                                  N)
    {
        using namespace cuda_tool;

        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if(tid >= N)
            return;

        int local_warp_id = threadIdx.x / BANKSIZE;
        int lane_id       = tid % BANKSIZE;

        __shared__ int cache_msk_s[DEFAULT_BLOCKSIZE];
        auto           cache_msk = make_dense_1d(cache_msk_s);

        int idx = part_to_real(tid);
        if(idx < 0)
            return;

        unsigned int prefix_msk = fine_connect_mask(idx);
        unsigned int conn_msk   = 0;
        int coarse_idx = coarse_table((level - 1) * vert_num + idx);
        if(coarse_idx < 0 || coarse_idx >= N)
        {
            return;
        }
        int kn       = neighbor_num(idx);
        int nk       = 0;
        int start_id = neighbor_start(idx);

        for(int i = 0; i < kn; i++)
        {
            unsigned int connect = neighbor_list(start_id + i);
            int coarse_connect = coarse_table((level - 1) * vert_num + connect);
            if(coarse_connect < 0 || coarse_connect >= N)
            {
                neighbor_list(start_id + nk) = connect;
                nk++;
                continue;
            }
            if(coarse_idx / BANKSIZE == coarse_connect / BANKSIZE)
            {
                conn_msk |= (1U << (coarse_connect % BANKSIZE));
            }
            else
            {
                neighbor_list(start_id + nk) = connect;
                nk++;
            }
        }
        neighbor_num(idx)      = nk;
        cache_msk(threadIdx.x) = 0;

        if(__popc(prefix_msk) == BANKSIZE)
        {
            atomicOr(&cache_msk(local_warp_id * BANKSIZE), static_cast<int>(conn_msk));
            conn_msk = static_cast<unsigned int>(cache_msk(local_warp_id * BANKSIZE));
        }
        else
        {
            unsigned int elected_lane = __ffs(prefix_msk) - 1;
            if(conn_msk)
                atomicOr(&cache_msk(local_warp_id * BANKSIZE + elected_lane),
                         static_cast<int>(conn_msk));
            conn_msk = static_cast<unsigned int>(cache_msk(local_warp_id * BANKSIZE + elected_lane));
        }

        unsigned int elected_prefix = __popc(prefix_msk & lanemask_lt(lane_id));
        if(conn_msk && elected_prefix == 0)
            atomicOr(&next_connect_mask(coarse_idx), conn_msk);
    }

    __global__ void MASPreconditionerEngine_next_level_cluster_kernel(
        cuda_tool::BufferView<unsigned int> next_connect_mask,
        cuda_tool::BufferView<unsigned int> next_prefix,
        int                                 N)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= N)
            return;

        int local_warp_id = threadIdx.x / BANKSIZE;
        int lane_id       = idx % BANKSIZE;

        __shared__ int          prefix_sum_raw[DEFAULT_WARPNUM];
        __shared__ unsigned int cached_msk_raw[DEFAULT_BLOCKSIZE];
        auto prefix_sum_s = make_dense_1d(prefix_sum_raw);
        auto cached_msk   = make_dense_1d(cached_msk_raw);

        if(lane_id == 0)
            prefix_sum_s(local_warp_id) = 0;

        unsigned int conn_msk = (1U << lane_id) | next_connect_mask(idx);
        cached_msk(threadIdx.x) = conn_msk;
        unsigned int visited    = (1U << lane_id);

        while(true)
        {
            unsigned int todo = visited ^ conn_msk;
            if(!todo)
                break;
            unsigned int next_visit = __ffs(todo) - 1;
            visited |= (1U << next_visit);
            conn_msk |= cached_msk(next_visit + local_warp_id * BANKSIZE);
        }

        next_connect_mask(idx) = conn_msk;
        unsigned int elected_prefix = __popc(conn_msk & lanemask_lt(lane_id));
        if(elected_prefix == 0)
            atomicAdd(&prefix_sum_s(local_warp_id), 1);
        if(lane_id == 0)
            next_prefix(idx / BANKSIZE) = prefix_sum_s(local_warp_id);
    }

    __global__ void MASPreconditionerEngine_prefix_sum_Lx_kernel(
        cuda_tool::BufferView<Int2>         level_size_ptr,
        cuda_tool::CBufferView<unsigned int>  next_prefix,
        cuda_tool::CBufferView<unsigned int>  next_prefix_sum,
        cuda_tool::BufferView<unsigned int>   next_connect_mask,
        cuda_tool::BufferView<int>            going_next_level,
        int                                   level,
        int                                   next_level_begin,
        int                                   N)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= N)
            return;

        int warp_id       = idx / BANKSIZE;
        int local_warp_id = threadIdx.x / BANKSIZE;
        int lane_id       = idx % BANKSIZE;

        __shared__ unsigned int elected_mask_s[BANKSIZE];
        __shared__ unsigned int lane_prefix_s[BANKSIZE * BANKSIZE];
        auto elected_mask = make_dense_1d(elected_mask_s);
        auto lane_prefix  = make_dense_1d(lane_prefix_s);

        if(lane_id == 0)
            elected_mask(local_warp_id) = 0;

        if(idx == N - 1)
        {
            level_size_ptr(level + 1).x =
                next_prefix_sum(warp_id) + next_prefix(warp_id);
            level_size_ptr(level + 1).y = next_level_begin;
        }

        unsigned int conn_msk = next_connect_mask(idx);
        unsigned int elected_prefix = __popc(conn_msk & lanemask_lt(lane_id));
        if(elected_prefix == 0)
            atomicOr(&elected_mask(local_warp_id), (1U << lane_id));

        lane_prefix(threadIdx.x) =
            __popc(elected_mask(local_warp_id) & lanemask_lt(lane_id));
        lane_prefix(threadIdx.x) += next_prefix_sum(warp_id);

        unsigned int elected_lane = __ffs(conn_msk) - 1;
        unsigned int the_lane_prefix =
            lane_prefix(elected_lane + BANKSIZE * local_warp_id);

        next_connect_mask(idx) = the_lane_prefix;
        going_next_level(idx) = the_lane_prefix + next_level_begin;
    }

    __global__ void MASPreconditionerEngine_compute_next_level_kernel(
        cuda_tool::BufferView<int>           coarse_table,
        cuda_tool::CBufferView<unsigned int> next_connect_mask,
        int                                  level,
        int                                  N)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= N)
            return;

        int next = coarse_table((level - 1) * N + idx);
        if(next < 0 || next >= N)
        {
            coarse_table(level * N + idx) = -1;
            return;
        }
        coarse_table(level * N + idx) = next_connect_mask(next);
    }

    __global__ void MASPreconditionerEngine_aggregation_kernel_kernel(
        cuda_tool::BufferView<LevelTable> coarse_table,
        cuda_tool::CBufferView<int>       going_next,
        cuda_tool::CBufferView<Int2>    level_size,
        int                               level_num,
        int                               n)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        int        current_id = idx;
        LevelTable ctable;

        int first = going_next(current_id);
        if(first >= 0)
        {
            UIPC_KERNEL_ASSERT(first >= level_size(1).y
                            && first < level_size(2).y,
                        "aggregation: going_next[%d]=%d not in level 1 [%d, %d)",
                        current_id, first,
                        level_size(1).y, level_size(2).y);

            current_id      = first;
            ctable.index[0] = first;

            for(int l = 1; l < level_num - 1; l++)
            {
                int next = going_next(current_id);

                UIPC_KERNEL_ASSERT(next >= 0,
                            "aggregation: partitioned vertex %d has "
                            "going_next=%d at level %d (expected >= 0)",
                            idx, next, l + 1);

                UIPC_KERNEL_ASSERT(next >= level_size(l + 1).y
                                && next < level_size(l + 2).y,
                            "aggregation: going_next[%d]=%d not in level %d [%d, %d)",
                            current_id, next, l + 1,
                            level_size(l + 1).y, level_size(l + 2).y);

                current_id      = next;
                ctable.index[l] = next;
            }
        }

        coarse_table(idx) = ctable;
    }

    __global__ void MASPreconditionerEngine_scatter_hessian_to_clusters_k1_kernel(
        int                                  offset,
        int                                  level_num,
        cuda_tool::CBufferView<int>          going_next,
        cuda_tool::CBufferView<Int2>       level_size,
        cuda_tool::BufferView<ClusterMatrixSym> cluster_hess,
        cuda_tool::CBufferView<int>          real_to_part,
        cuda_tool::CBufferView<uint32_t>     indices,
        cuda_tool::CBufferView<Eigen::Matrix3d> triplet_values,
        cuda_tool::CBufferView<int>          row_ids,
        cuda_tool::CBufferView<int>          col_ids,
        int                                  total_nodes,
        int                                  n)
    {
        using namespace cuda_tool;

        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;

        int  index    = indices(I);
        int  row_real = row_ids(index) - offset;
        int  col_real = col_ids(index) - offset;
        auto H        = triplet_values(index);

        if(row_real < 0 || row_real >= total_nodes || col_real < 0 || col_real >= total_nodes)
            return;

        int vert_col = real_to_part(col_real);
        int vert_row = real_to_part(row_real);

        if(vert_col < 0 || vert_row < 0)
            return;

        if(vert_col / BANKSIZE == vert_row / BANKSIZE)
        {
            int cluster_id = vert_col / BANKSIZE;
            if(vert_col >= vert_row)
            {
                int si = sym_index(vert_row % BANKSIZE, vert_col % BANKSIZE);
                cuda_tool::eigen::atomic_add(cluster_hess(cluster_id).M[si], H);
            }
            else
            {
                Eigen::Matrix3d Ht = H.transpose();
                int si = sym_index(vert_col % BANKSIZE, vert_row % BANKSIZE);
                cuda_tool::eigen::atomic_add(cluster_hess(cluster_id).M[si], Ht);
            }
        }
        else
        {
            // Walk up ALL levels and add at every level where both endpoints
            // land in the same bank. This implements Galerkin H_L = R_L H R_L^T
            // independently per level (an entry can contribute to L1, L2, L3, ...
            // wherever its ancestors' banks merge).
            int level = 0;
            while(level < level_num - 1)
            {
                level++;
                if(level == 1)
                {
                    vert_col = going_next(col_real);
                    vert_row = going_next(row_real);
                }
                else
                {
                    vert_col = going_next(vert_col);
                    vert_row = going_next(vert_row);
                }
                if(vert_col < 0 || vert_row < 0)
                    return;

                UIPC_KERNEL_ASSERT(vert_col >= level_size(level).y
                                && vert_col < level_size(level + 1).y,
                            "scatter P1: vert_col=%d not in level %d [%d, %d)",
                            vert_col, level, level_size(level).y, level_size(level + 1).y);
                UIPC_KERNEL_ASSERT(vert_row >= level_size(level).y
                                && vert_row < level_size(level + 1).y,
                            "scatter P1: vert_row=%d not in level %d [%d, %d)",
                            vert_row, level, level_size(level).y, level_size(level + 1).y);

                if(vert_col / BANKSIZE == vert_row / BANKSIZE)
                {
                    int cluster_id = vert_col / BANKSIZE;
                    if(vert_col >= vert_row)
                    {
                        int si = sym_index(vert_row % BANKSIZE, vert_col % BANKSIZE);
                        for(int i = 0; i < 3; i++)
                            for(int j = 0; j < 3; j++)
                            {
                                atomicAdd(&(cluster_hess(cluster_id).M[si](i, j)),
                                          H(i, j));
                                if(vert_col == vert_row)
                                    atomicAdd(&(cluster_hess(cluster_id).M[si](i, j)),
                                              H(j, i));
                            }
                    }
                    else
                    {
                        int si = sym_index(vert_col % BANKSIZE, vert_row % BANKSIZE);
                        for(int i = 0; i < 3; i++)
                            for(int j = 0; j < 3; j++)
                                atomicAdd(&(cluster_hess(cluster_id).M[si](i, j)),
                                          H(j, i));
                    }
                    // NO break: keep walking up to coarser levels where
                    // banks may also merge. Each level's Galerkin restriction
                    // is independent (R_L H R_L^T).
                }
            }
        }
    }

    __global__ void MASPreconditionerEngine_scatter_hessian_to_clusters_k2_kernel(
        int                               level_num,
        cuda_tool::CBufferView<int>       going_next,
        cuda_tool::CBufferView<Int2>    level_size,
        cuda_tool::BufferView<ClusterMatrixSym> cluster_hess,
        cuda_tool::CBufferView<int>       part_to_real,
        cuda_tool::CBufferView<unsigned int> fine_connect,
        cuda_tool::CBufferView<int>       prefix_orig,
        int                               map_nodes,
        int                               n)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        int cluster_stride = BANKSIZE * BANKSIZE;
        int cluster_id     = idx / cluster_stride;
        int local_row      = (idx % cluster_stride) / BANKSIZE;
        int local_col      = (idx % cluster_stride) % BANKSIZE;

        int global_row = cluster_id * BANKSIZE + local_row;
        int global_col = cluster_id * BANKSIZE + local_col;
        if(global_row >= map_nodes || global_col >= map_nodes)
            return;

        int rdx = part_to_real(global_row);
        int cdx = part_to_real(global_col);

        __shared__ int prefix;
        if(threadIdx.x == 0)
            prefix = prefix_orig(cluster_id);
        __syncthreads();

        Eigen::Matrix3d mat3;
        if(local_col >= local_row)
        {
            int si = sym_index(local_row, local_col);
            mat3   = cluster_hess(cluster_id).M[si];
        }
        else
        {
            int si = sym_index(local_col, local_row);
            mat3   = cluster_hess(cluster_id).M[si].transpose();
        }

        // Do NOT early-return for invalid lanes — downstream code has warp-
        // scope collectives (cub::WarpReduce, atomicAdd chain) that require
        // every lane in the hw warp to be active. Instead, zero out mat3 so
        // invalid lanes contribute nothing to the reduction.
        bool invalid = (rdx < 0) || (cdx < 0);
        if(invalid)
            mat3.setZero();

        if(prefix == 1)
        {
            // Full 32-lane reduction per hw warp; mat3 for invalid lanes = 0.
            using WarpReduceD = cub::WarpReduce<double>;
            __shared__
                typename WarpReduceD::TempStorage temp_reduce_d[BANKSIZE * BANKSIZE / 32];
            int hw_warp = threadIdx.x / 32;

            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    mat3(i, j) =
                        WarpReduceD(temp_reduce_d[hw_warp]).Sum(mat3(i, j));

            // Lane 0 of each hw warp writes the warp's sum to coarser levels,
            // but only if its own lane was valid (rdx >= 0).
            if((threadIdx.x & 0x1f) == 0 && !invalid)
            {
                int level   = 0;
                int next_id = going_next(rdx);
                while(next_id >= 0 && level < level_num - 1)
                {
                    level++;

                    UIPC_KERNEL_ASSERT(next_id >= level_size(level).y
                                    && next_id < level_size(level + 1).y,
                                "scatter P2 diag: next_id=%d not in level %d [%d, %d)",
                                next_id, level, level_size(level).y, level_size(level + 1).y);

                    int cid = next_id / BANKSIZE;
                    int bv  = next_id % BANKSIZE;
                    int si  = sym_index(bv, bv);
                    for(int i = 0; i < 3; i++)
                        for(int j = 0; j < 3; j++)
                            atomicAdd(&(cluster_hess(cid).M[si](i, j)),
                                      mat3(i, j));
                    next_id = going_next(next_id);
                }
            }
        }
        else
        {
            // prefix > 1: each lane walks up independently, no warp collective.
            if(invalid)
                return;

            int level = 1;
            while(level <= level_num - 1)
            {
                rdx = going_next(rdx);
                cdx = going_next(cdx);
                if(rdx < 0 || cdx < 0)
                    return;

                UIPC_KERNEL_ASSERT(rdx >= level_size(level).y
                                && rdx < level_size(level + 1).y,
                            "scatter P2: rdx=%d not in level %d [%d, %d)",
                            rdx, level, level_size(level).y, level_size(level + 1).y);
                UIPC_KERNEL_ASSERT(cdx >= level_size(level).y
                                && cdx < level_size(level + 1).y,
                            "scatter P2: cdx=%d not in level %d [%d, %d)",
                            cdx, level, level_size(level).y, level_size(level + 1).y);

                int cid = cdx / BANKSIZE;
                if(rdx / BANKSIZE == cdx / BANKSIZE)
                {
                    if(cdx >= rdx)
                    {
                        int si = sym_index(rdx % BANKSIZE, cdx % BANKSIZE);
                        for(int i = 0; i < 3; i++)
                            for(int j = 0; j < 3; j++)
                                atomicAdd(&(cluster_hess(cid).M[si](i, j)),
                                          mat3(i, j));
                    }
                }
                level++;
            }
        }
    }

    __global__ void MASPreconditionerEngine_invert_cluster_matrices_kernel(
        cuda_tool::BufferView<ClusterMatrixSymF> cluster_inv,
        cuda_tool::CBufferView<ClusterMatrixSym> cluster_hess,
        int                                      total_threads)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= total_threads)
            return;

        constexpr int MAT_DIM = BANKSIZE * 3;  // 48

        int mat_id       = idx / MAT_DIM;
        int col          = idx % MAT_DIM;
        int block_mat_id = threadIdx.x / MAT_DIM;

        __shared__ double s_mat_raw[32 / BANKSIZE][MAT_DIM][MAT_DIM];
        __shared__ double s_col_raw[32 / BANKSIZE][MAT_DIM];
        auto s_mat = make_dense_2d(&s_mat_raw[block_mat_id][0][0], MAT_DIM, MAT_DIM);
        auto s_col = make_dense_1d(&s_col_raw[block_mat_id][0], MAT_DIM);

        for(int row = 0; row < MAT_DIM; row++)
        {
            int node_row = row / 3;
            int node_col = col / 3;
            if(node_col >= node_row)
            {
                int si = sym_index(node_row, node_col);
                s_mat(row, col) = cluster_hess(mat_id).M[si](row % 3, col % 3);
            }
            else
            {
                int si = sym_index(node_col, node_row);
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
                    __syncthreads();
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

    __global__ void MASPreconditionerEngine_build_multi_level_R_kernel(
        cuda_tool::CDenseVectorView<Float>     R_view,
        cuda_tool::BufferView<Eigen::Vector3f> multi_lr,
        cuda_tool::CBufferView<int>            going_next,
        cuda_tool::CBufferView<Int2>         level_size,
        cuda_tool::CBufferView<int>            prefix_orig,
        cuda_tool::CBufferView<unsigned int>   fine_conn,
        cuda_tool::CBufferView<int>            part_to_real,
        cuda_tool::CDense<IndexT>              converged,
        int                                    level_num,
        int                                    N)
    {
        using namespace cuda_tool;

        if(*converged != 0)
            return;
        int pdx = blockIdx.x * blockDim.x + threadIdx.x;

        bool in_range = (pdx < N);
        int  idx      = in_range ? part_to_real(pdx) : -1;
        bool is_valid = (idx >= 0);

        Eigen::Vector3f r = Eigen::Vector3f::Zero();
        if(is_valid)
        {
            auto seg = R_view.segment<3>(3 * idx);
            r[0]     = static_cast<float>(seg(0));
            r[1]     = static_cast<float>(seg(1));
            r[2]     = static_cast<float>(seg(2));
        }

        int lane_id       = threadIdx.x % BANKSIZE;
        int local_warp_id = threadIdx.x / BANKSIZE;
        int global_warp   = pdx / BANKSIZE;

        if(in_range)
            multi_lr(pdx) = r;

        __shared__ float sum_residual_s[DEFAULT_BLOCKSIZE * 3];
        __shared__ int   prefix_sum_raw[DEFAULT_WARPNUM];
        auto             sum_residual = make_dense_1d(sum_residual_s);
        auto             prefix_sum_v = make_dense_1d(prefix_sum_raw);

        if(lane_id == 0)
            prefix_sum_v(local_warp_id) = in_range ? prefix_orig(global_warp) : 0;

        unsigned int connect_msk = is_valid ? fine_conn(idx) : 0U;

        if(prefix_sum_v(local_warp_id) == 1)
        {
            using WarpReduceF = cub::WarpReduce<float, BANKSIZE>;
            __shared__ typename WarpReduceF::TempStorage temp_reduce_f[DEFAULT_WARPNUM];

            r[0] = WarpReduceF(temp_reduce_f[local_warp_id]).Sum(r[0]);
            r[1] = WarpReduceF(temp_reduce_f[local_warp_id]).Sum(r[1]);
            r[2] = WarpReduceF(temp_reduce_f[local_warp_id]).Sum(r[2]);

            if(lane_id == 0 && is_valid)
            {
                int cur = idx;
                for(int l = 0; l < level_num - 1; l++)
                {
                    cur = going_next(cur);

                    UIPC_KERNEL_ASSERT(cur >= level_size(l + 1).y
                                    && cur < level_size(l + 2).y,
                                "build_R: cur=%d not in level %d [%d, %d)",
                                cur, l + 1, level_size(l + 1).y, level_size(l + 2).y);

                    atomicAdd(&(multi_lr(cur)[0]), r[0]);
                    atomicAdd(&(multi_lr(cur)[1]), r[1]);
                    atomicAdd(&(multi_lr(cur)[2]), r[2]);
                }
            }
        }
        else if(is_valid)
        {
            int elected_lane = __ffs(connect_msk) - 1;

            sum_residual(threadIdx.x)                         = 0;
            sum_residual(threadIdx.x + DEFAULT_BLOCKSIZE)     = 0;
            sum_residual(threadIdx.x + 2 * DEFAULT_BLOCKSIZE) = 0;

            atomicAdd(&sum_residual(local_warp_id * BANKSIZE + elected_lane),
                      r[0]);
            atomicAdd(&sum_residual(local_warp_id * BANKSIZE + elected_lane + DEFAULT_BLOCKSIZE),
                      r[1]);
            atomicAdd(&sum_residual(local_warp_id * BANKSIZE + elected_lane
                                    + 2 * DEFAULT_BLOCKSIZE),
                      r[2]);

            unsigned int elected_prefix =
                __popc(connect_msk & lanemask_lt(lane_id));
            if(elected_prefix == 0)
            {
                int cur = idx;
                for(int l = 0; l < level_num - 1; l++)
                {
                    cur = going_next(cur);

                    UIPC_KERNEL_ASSERT(cur >= level_size(l + 1).y
                                    && cur < level_size(l + 2).y,
                                "build_R: cur=%d not in level %d [%d, %d)",
                                cur, l + 1, level_size(l + 1).y, level_size(l + 2).y);

                    atomicAdd(&(multi_lr(cur)[0]),
                              sum_residual(threadIdx.x));
                    atomicAdd(&(multi_lr(cur)[1]),
                              sum_residual(threadIdx.x + DEFAULT_BLOCKSIZE));
                    atomicAdd(&(multi_lr(cur)[2]),
                              sum_residual(threadIdx.x + DEFAULT_BLOCKSIZE * 2));
                }
            }
        }
    }

    __global__ void MASPreconditionerEngine_schwarz_local_solve_kernel(
        cuda_tool::CBufferView<ClusterMatrixSymF> cluster_inv,
        cuda_tool::CBufferView<Eigen::Vector3f>   multi_lr,
        cuda_tool::BufferView<float3>             multi_lz,
        cuda_tool::CDense<IndexT>                 converged,
        int                                       N)
    {
        using namespace cuda_tool;

        if(*converged != 0)
            return;
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= N)
            return;

        constexpr int cluster_stride = BANKSIZE * BANKSIZE;

        int cluster_id = idx / cluster_stride;
        int local_row  = (idx % cluster_stride) / BANKSIZE;
        int local_col  = (idx % cluster_stride) % BANKSIZE;

        int vert_row = cluster_id * BANKSIZE + local_row;
        int vert_col = cluster_id * BANKSIZE + local_col;

        __shared__ Eigen::Vector3f s_R_raw[BANKSIZE];
        auto                       s_R = make_dense_1d(s_R_raw);
        if(threadIdx.x < BANKSIZE)
            s_R(threadIdx.x) = multi_lr(vert_col);
        __syncthreads();

        Eigen::Vector3f result;
        if(vert_col >= vert_row)
        {
            int si = sym_index(local_row, local_col);
            result = cluster_inv(cluster_id).M[si] * s_R(local_col);
        }
        else
        {
            int si = sym_index(local_col, local_row);
            result = cluster_inv(cluster_id).M[si].transpose() * s_R(local_col);
        }

        using WarpReduceF = cub::WarpReduce<float, BANKSIZE>;
        __shared__ typename WarpReduceF::TempStorage temp_reduce_f[BANKSIZE];
        int logical_warp = threadIdx.x / BANKSIZE;

        result[0] = WarpReduceF(temp_reduce_f[logical_warp]).Sum(result[0]);
        result[1] = WarpReduceF(temp_reduce_f[logical_warp]).Sum(result[1]);
        result[2] = WarpReduceF(temp_reduce_f[logical_warp]).Sum(result[2]);

        if((threadIdx.x % BANKSIZE) == 0)
        {
            atomicAdd(&(multi_lz(vert_row).x), result[0]);
            atomicAdd(&(multi_lz(vert_row).y), result[1]);
            atomicAdd(&(multi_lz(vert_row).z), result[2]);
        }
    }

    __global__ void MASPreconditionerEngine_collect_final_Z_kernel(
        cuda_tool::DenseVectorView<Float>  Z_view,
        cuda_tool::CBufferView<float3>     multi_lz,
        cuda_tool::CBufferView<LevelTable> coarse_table,
        cuda_tool::CBufferView<int>        real_to_part,
        cuda_tool::CDense<IndexT>          converged,
        int                                level_num,
        int                                n)
    {
        using namespace cuda_tool;

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        if(*converged != 0)
            return;
        int rdx = real_to_part(idx);

        // Skip unpartitioned vertices (handled by diagonal fallback)
        if(rdx < 0)
            return;

        // Start with the fine-level solution
        float3 cz = multi_lz(rdx);

        // Pure injection prolongation: sum coarse Z contributions directly,
        // matching StiffGIPC's __collectFinalZ_new (no scaling).
        // The Galerkin coarsening (H_coarse = R H R^T) is consistent with
        // the summation restriction, so no additional scaling is needed.
        LevelTable table = coarse_table(idx);
        for(int l = 1; l < level_num; l++)
        {
            int    node = table.index[l - 1];
            float3 val  = multi_lz(node);
            cz.x += val.x;
            cz.y += val.y;
            cz.z += val.z;
        }

        auto seg = Z_view.segment<3>(3 * idx);
        seg(0)   = static_cast<double>(cz.x);
        seg(1)   = static_cast<double>(cz.y);
        seg(2)   = static_cast<double>(cz.z);
    }
}  // namespace

// ============================================================================
// Phase 1: Initialization
// ============================================================================

void MASPreconditionerEngine::compute_num_levels(int vert_num)
{
    int n_level  = 1;
    int level_sz = bank_align(vert_num);

    while(level_sz > BANKSIZE)
    {
        level_sz /= BANKSIZE;
        n_level++;
        level_sz = bank_align(level_sz);
    }
    n_level++;
    m_level_num        = std::min(n_level, MAX_LEVELS);
    m_active_level_num = 0;  // 0 = use full hierarchy (reset on each init_matrix)
}

void MASPreconditionerEngine::init_neighbor(int                     vert_num,
                                            int                     total_neighbor_num,
                                            int                     part_map_size,
                                            span<const unsigned int> h_neighbor_list,
                                            span<const unsigned int> h_neighbor_start,
                                            span<const unsigned int> h_neighbor_num,
                                            span<const int>          h_part_to_real,
                                            span<const int>          h_real_to_part)
{
    if(vert_num < 1)
        return;

    int max_nodes    = std::max(part_map_size, vert_num);
    int padded_nodes = bank_align(max_nodes);
    compute_num_levels(max_nodes);

    m_total_map_nodes = part_map_size;
    m_total_nodes     = vert_num;

    // Hierarchy buffers
    dense_level.resize(vert_num);
    real_to_part.resize(vert_num);
    coarse_tables.resize(vert_num);
    coarse_space_tables.resize(vert_num * m_level_num);
    level_sizes.resize(m_level_num + 1);
    going_next.resize(padded_nodes * m_level_num);
    prefix_original.resize(max_nodes);
    next_prefixes.resize(max_nodes);
    next_prefix_sums.resize(max_nodes);
    prefix_sum_original.resize(max_nodes);
    fine_connect_masks.resize(max_nodes);
    next_connect_masks.resize(max_nodes);

    // Neighbor buffers
    m_neighbor_list_size = total_neighbor_num;
    neighbor_lists.resize(total_neighbor_num);
    neighbor_starts.resize(vert_num);
    neighbor_nums.resize(vert_num);
    neighbor_lists_init.resize(total_neighbor_num);
    neighbor_nums_init.resize(vert_num);

    // Partition mappings
    part_to_real.resize(part_map_size);

    // Upload host data
    neighbor_lists_init.view().copy_from(h_neighbor_list.data());
    neighbor_starts.view().copy_from(h_neighbor_start.data());
    neighbor_nums_init.view().copy_from(h_neighbor_num.data());
    part_to_real.view().copy_from(h_part_to_real.data());
    real_to_part.view().copy_from(h_real_to_part.data());
}

void MASPreconditionerEngine::init_matrix()
{
    if(m_total_nodes < 1)
        return;

    // Restore neighbor data for initial hierarchy build
    neighbor_lists.view().copy_from(neighbor_lists_init.view());
    neighbor_nums.view().copy_from(neighbor_nums_init.view());

    int total_cluster = static_cast<int>(reorder_realtime(0) * 1.05);
    int num_blocks    = total_cluster / BANKSIZE;

    cluster_hessians.resize(num_blocks);
    cluster_inverses.resize(num_blocks);
    multi_level_R.resize(total_cluster);
    multi_level_Z.resize(total_cluster);

    m_initialized = true;
}

// ============================================================================
// Hierarchy building
// ============================================================================

int MASPreconditionerEngine::reorder_realtime(int cp_num)
{
    level_sizes.fill(Int2{0, 0});
    coarse_space_tables.fill(-1);
    going_next.fill(-1);

    build_connect_mask_L0();
    prepare_prefix_sum_L0();
    build_level1();

    for(int level = 1; level < m_level_num; level++)
    {
        next_connect_masks.fill(0u);
        build_connect_mask_Lx(level);

        level_sizes.view(level, 1).copy_to(&m_h_level_size);

        next_level_cluster(level);
        prefix_sum_Lx(level);
        compute_next_level(level);
    }

    level_sizes.view(m_level_num, 1).copy_to(&m_h_level_size);

    m_total_num_clusters = m_h_level_size.y;
    aggregation_kernel();

    return m_total_num_clusters;
}

// ---------------------------------------------------------------------------
// Build connectivity mask at level 0 (fine level)
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_connect_mask_L0()
{
    using namespace cuda_tool;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    auto k = MASPreconditionerEngine_build_connect_mask_L0_kernel;
    k<<<cuda_tool::best_grid_dim(N, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
        neighbor_starts.cview(),
        neighbor_nums.view(),
        neighbor_lists.view(),
        fine_connect_masks.view(),
        part_to_real.cview(),
        real_to_part.cview(),
        N);
}

// ---------------------------------------------------------------------------
// Prepare prefix sum at level 0 (uses shared memory)
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::prepare_prefix_sum_L0()
{
    using namespace cuda_tool;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    MASPreconditionerEngine_prepare_prefix_sum_L0_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        fine_connect_masks.view(), prefix_original.view(), part_to_real.cview(), N);
}

// ---------------------------------------------------------------------------
// Build coarse level 1 from prefix sums
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_level1()
{
    using namespace cuda_tool;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = BANKSIZE * BANKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;
    int warp_num   = (N + BANKSIZE - 1) / BANKSIZE;

    thrust::exclusive_scan(thrust::device_ptr<int>(prefix_original.data()),
                           thrust::device_ptr<int>(prefix_original.data()) + warp_num,
                           thrust::device_ptr<int>(prefix_sum_original.data()));

    // Cluster matrices are indexed by  node_index / BANKSIZE.
    // Level-0 uses partition indices [0, N),  so its cluster IDs occupy
    // [0, ceil(N / BANKSIZE)).
    //
    // padded_N = align(N, BANKSIZE)
    //   — round N up to a BANKSIZE boundary so that every level-0 bank
    //     is a full block; level-1 indices must start no earlier than this
    //     to avoid sharing a cluster ID with any level-0 bank.
    //
    // max(padded_N, m_total_nodes)
    //   — when unpartitioned vertices exist,
    //     m_total_nodes > m_total_map_nodes is possible.  going_next is
    //     read at real-vertex indices [0, m_total_nodes), so level-1
    //     indices must also exceed m_total_nodes.
    //
    // align(..., BANKSIZE)
    //   — the outer align keeps the level-1 region BANKSIZE-aligned,
    //     which is required by the bank-based cluster addressing used
    //     in all subsequent kernels.
    int padded_N     = bank_align(N);
    int level1_begin = bank_align(std::max(padded_N, m_total_nodes));

    MASPreconditionerEngine_build_level1_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        level_sizes.view(),
        coarse_space_tables.view(),
        going_next.view(0, level1_begin),
        fine_connect_masks.cview(),
        prefix_sum_original.cview(),
        prefix_original.cview(),
        part_to_real.cview(),
        N,
        level1_begin);
}

// ---------------------------------------------------------------------------
// Build connectivity mask at level x (coarsened)
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_connect_mask_Lx(int level)
{
    using namespace cuda_tool;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    MASPreconditionerEngine_build_connect_mask_Lx_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        neighbor_starts.cview(),
        neighbor_nums.view(),
        neighbor_lists.view(),
        coarse_space_tables.cview(),
        next_connect_masks.view(),
        fine_connect_masks.cview(),
        part_to_real.cview(),
        level,
        m_total_nodes,
        N);
}

// ---------------------------------------------------------------------------
// Cluster connectivity at next level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::next_level_cluster(int level)
{
    using namespace cuda_tool;
    int N = m_h_level_size.x;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    MASPreconditionerEngine_next_level_cluster_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        next_connect_masks.view(), next_prefixes.view(), N);
}

// ---------------------------------------------------------------------------
// Prefix sum at level x
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::prefix_sum_Lx(int level)
{
    using namespace cuda_tool;
    int N = m_h_level_size.x;
    if(N < 1)
        return;

    int level_begin       = m_h_level_size.y;
    int level_region_size = bank_align(N);
    int next_level_begin  = level_begin + level_region_size;
    int block_size        = BANKSIZE * BANKSIZE;
    int num_blocks        = (N + block_size - 1) / block_size;
    int warp_num          = (N + BANKSIZE - 1) / BANKSIZE;

    thrust::exclusive_scan(thrust::device_ptr<unsigned int>(next_prefixes.data()),
                           thrust::device_ptr<unsigned int>(next_prefixes.data()) + warp_num,
                           thrust::device_ptr<unsigned int>(next_prefix_sums.data()));

    MASPreconditionerEngine_prefix_sum_Lx_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        level_sizes.view(),
        next_prefixes.cview(),
        next_prefix_sums.cview(),
        next_connect_masks.view(),
        going_next.view(level_begin, level_region_size),
        level,
        next_level_begin,
        N);
}

// ---------------------------------------------------------------------------
// Compute next coarsening level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::compute_next_level(int level)
{
    using namespace cuda_tool;
    int N = m_total_nodes;
    if(N < 1)
        return;

    auto k = MASPreconditionerEngine_compute_next_level_kernel;
    k<<<cuda_tool::best_grid_dim(N, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
        coarse_space_tables.view(), next_connect_masks.cview(), level, N);
}

// ---------------------------------------------------------------------------
// Build per-node level traversal table
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::aggregation_kernel()
{
    using namespace cuda_tool;
    int N = m_total_nodes;
    if(N < 1 || m_total_num_clusters < 1)
        return;

    int level_num = m_level_num;
    auto k        = MASPreconditionerEngine_aggregation_kernel_kernel;
    k<<<cuda_tool::best_grid_dim(N, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
        coarse_tables.view(),
        going_next.cview(0, m_total_num_clusters),
        level_sizes.cview(),
        level_num,
        N);
}

// ============================================================================
// Phase 2: Assemble preconditioner
// ============================================================================

void MASPreconditionerEngine::set_preconditioner(cuda_tool::CBufferView<Eigen::Matrix3d> triplet_values,
                                                 cuda_tool::CBufferView<int>             row_ids,
                                                 cuda_tool::CBufferView<int>             col_ids,
                                                 cuda_tool::CBufferView<uint32_t>        indices,
                                                 int                                dof_offset,
                                                 int                                cp_num)
{
    if(m_total_nodes < 1)
        return;

    // 1. Restore neighbor data for this iteration
    neighbor_lists.view().copy_from(neighbor_lists_init.view());
    neighbor_nums.view().copy_from(neighbor_nums_init.view());

    // 2. Rebuild multi-level hierarchy
    reorder_realtime(cp_num);

    // 3. Resize cluster matrices if needed
    int num_cluster_blocks = m_total_num_clusters / BANKSIZE;
    if(num_cluster_blocks < 1)
        return;

    if(num_cluster_blocks > static_cast<int>(cluster_hessians.size()))
    {
        cluster_hessians.resize(num_cluster_blocks);
        cluster_inverses.resize(num_cluster_blocks);
    }

    // Resize multi-level buffers if needed
    if(m_total_num_clusters > static_cast<int>(multi_level_R.size()))
    {
        multi_level_R.resize(m_total_num_clusters);
        multi_level_Z.resize(m_total_num_clusters);
    }

    cluster_hessians.view(0, num_cluster_blocks).fill(ClusterMatrixSym{});

    // 5. Scatter BCOO Hessian blocks into cluster matrices
    scatter_hessian_to_clusters(triplet_values, row_ids, col_ids, indices, dof_offset);

    // 6. Invert each cluster matrix (Gauss-Jordan)
    invert_cluster_matrices();
}

// ---------------------------------------------------------------------------
// Scatter BCOO Hessian entries into cluster-level dense matrices
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::scatter_hessian_to_clusters(cuda_tool::CBufferView<Eigen::Matrix3d> triplet_values,
                                                          cuda_tool::CBufferView<int>             row_ids,
                                                          cuda_tool::CBufferView<int>             col_ids,
                                                          cuda_tool::CBufferView<uint32_t>        indices,
                                                          int dof_offset)
{
    using namespace cuda_tool;

    int triplet_num = static_cast<int>(indices.size());

    // --- Pass 1: Place each 3x3 block at the finest level where both
    //             row and col belong to the same cluster. ---

    if(triplet_num > 0)
    {
        auto k = MASPreconditionerEngine_scatter_hessian_to_clusters_k1_kernel;
        k<<<cuda_tool::best_grid_dim(triplet_num, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dof_offset,
            m_level_num,
            going_next.cview(0, m_total_num_clusters),
            level_sizes.cview(),
            cluster_hessians.view(),
            real_to_part.cview(),
            indices,
            triplet_values,
            row_ids,
            col_ids,
            m_total_nodes,
            triplet_num);
    }

    // --- Pass 2: Scatter fine-level cluster matrices to coarser levels
    //             using warp-reduction for contiguous partitions. ---

    int total_entries = m_total_map_nodes * BANKSIZE;
    if(total_entries < 1)
        return;
    int thread_num = BANKSIZE * BANKSIZE;
    int block_num  = (total_entries + thread_num - 1) / thread_num;

    MASPreconditionerEngine_scatter_hessian_to_clusters_k2_kernel<<<block_num, thread_num, 0, nullptr>>>(
        m_level_num,
        going_next.cview(0, m_total_num_clusters),
        level_sizes.cview(),
        cluster_hessians.view(),
        part_to_real.cview(),
        fine_connect_masks.cview(),
        prefix_original.cview(),
        m_total_map_nodes,
        total_entries);
}

// ---------------------------------------------------------------------------
// Gauss-Jordan inversion of each 48x48 cluster matrix
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::invert_cluster_matrices()
{
    using namespace cuda_tool;
    int total_threads = m_total_num_clusters * 3;  // 48 threads per cluster
    if(total_threads < 1)
        return;

    int block_size = 32 * 3;  // 96 threads = 2 clusters per block
    int num_blocks = (total_threads + block_size - 1) / block_size;

    MASPreconditionerEngine_invert_cluster_matrices_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        cluster_inverses.view(), cluster_hessians.cview(), total_threads);
}

// ============================================================================
// Phase 3: Apply preconditioning  z = M^{-1} r
// ============================================================================

// ---------------------------------------------------------------------------
// Restrict: accumulate residual R from fine to all coarser levels
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_multi_level_R(cuda_tool::CDenseVectorView<Float> R,
                                                  cuda_tool::CVarView<IndexT> converged)
{
    using namespace cuda_tool;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    MASPreconditionerEngine_build_multi_level_R_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        R,
        multi_level_R.view(),
        going_next.cview(0, m_total_num_clusters),
        level_sizes.cview(),
        prefix_original.cview(),
        fine_connect_masks.cview(),
        part_to_real.cview(),
        converged.cviewer(),
        m_level_num,
        N);
}

// ---------------------------------------------------------------------------
// Local solve: Z = cluster_inverse * R at each level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::schwarz_local_solve(cuda_tool::CVarView<IndexT> converged)
{
    using namespace cuda_tool;
    int N = m_total_num_clusters * BANKSIZE;  // one thread per (cluster, node-pair)
    if(N < 1)
        return;

    int block_size = BANKSIZE * BANKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    MASPreconditionerEngine_schwarz_local_solve_kernel<<<num_blocks, block_size, 0, nullptr>>>(
        cluster_inverses.cview(),
        multi_level_R.cview(),
        multi_level_Z.view(),
        converged.cviewer(),
        N);
}

// ---------------------------------------------------------------------------
// Prolongate: sum Z contributions from all levels for each fine node
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::collect_final_Z(cuda_tool::DenseVectorView<Float> Z,
                                              cuda_tool::CVarView<IndexT> converged)
{
    using namespace cuda_tool;
    int N = m_total_nodes;
    if(N < 1)
        return;

    int level_num = (m_active_level_num > 0) ? m_active_level_num : m_level_num;
    auto k        = MASPreconditionerEngine_collect_final_Z_kernel;
    k<<<cuda_tool::best_grid_dim(N, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
        Z,
        multi_level_Z.cview(),
        coarse_tables.cview(),
        real_to_part.cview(),
        converged.cviewer(),
        level_num,
        N);
}

// ============================================================================
// Apply: full preconditioning pipeline  z = M^{-1} r
// ============================================================================

void MASPreconditionerEngine::apply(cuda_tool::CDenseVectorView<Float> r,
                                    cuda_tool::DenseVectorView<Float>  z,
                                    cuda_tool::CVarView<IndexT>        converged)
{
    if(m_total_nodes < 1)
        return;

    // Ensure multi-level buffers cover all clusters
    if(m_total_num_clusters > static_cast<int>(multi_level_R.size()))
    {
        multi_level_R.resize(m_total_num_clusters);
        multi_level_Z.resize(m_total_num_clusters);
    }

    if(m_total_num_clusters > m_total_map_nodes)
    {
        multi_level_R
            .view(m_total_map_nodes, m_total_num_clusters - m_total_map_nodes)
            .fill(Eigen::Vector3f::Zero());
    }

    multi_level_Z.view(0, m_total_num_clusters).fill(float3{0, 0, 0});

    // 1. Restrict: accumulate residual down through levels
    build_multi_level_R(r, converged);

    // 2. Local solve: Z = cluster_inverse * R at each level
    schwarz_local_solve(converged);

    // 3. Prolongate: sum Z from all levels back to fine nodes
    collect_final_Z(z, converged);
}

void MASPreconditionerEngine::dump_cluster_matrices_debug(std::string_view output_dir,
                                                          SizeT            frame,
                                                          SizeT            newton_iter)
{
    if(!m_initialized || cluster_hessians.size() == 0)
        return;

    cuda_tool::wait_device();

    // Materialize the path once for the lambdas / std::ofstream consumers below.
    const std::filesystem::path output_dir_path{output_dir};

    const size_t nb = cluster_hessians.size();
    std::vector<ClusterMatrixSym>  h_hess(nb);
    std::vector<ClusterMatrixSymF> h_inv(nb);

    cluster_hessians.view(0, nb).copy_to(h_hess.data());
    cluster_inverses.view(0, nb).copy_to(h_inv.data());

    // Block-upper-triangle dump: coordinate real general, (nb*48) × (nb*48) block-diagonal.
    // Each cluster writes SYM_BLOCK_COUNT full 3×3 blocks at their upper-triangle positions.
    // Diagonal blocks are full 3×3 (not scalar-upper-triangle), matching GPU storage exactly.
    // Uses fmt::memory_buffer + FILE* pattern consistent with utils/matrix_market.h.
    constexpr int DIM       = BANKSIZE * 3;
    const int     mat_size  = static_cast<int>(nb) * DIM;
    const int64_t total_nnz = static_cast<int64_t>(nb) * SYM_BLOCK_COUNT * 9;

    auto write_mtx = [&]<typename Scalar>(const std::vector<ClusterMatrixSymT<Scalar>>& clusters,
                                          std::string_view                              kind)
    {
        auto path =
            output_dir_path
            / fmt::format("mas_cluster_{}.f{}.n{}.mtx", kind, frame, newton_iter);
        auto path_str = path.string();

        auto buf = fmt::memory_buffer();

        fmt::format_to(std::back_inserter(buf),
                       "%%MatrixMarket matrix coordinate real general\n"
                       "% MAS cluster {} block-upper-triangle ({} clusters, banksize={})\n"
                       "{} {} {}\n",
                       kind,
                       nb,
                       BANKSIZE,
                       mat_size,
                       mat_size,
                       total_nnz);

        for(size_t c = 0; c < clusters.size(); c++)
        {
            const int base = static_cast<int>(c) * DIM;

            for(int br = 0; br < BANKSIZE; br++)
            {
                for(int bc = br; bc < BANKSIZE; bc++)
                {
                    int         k   = BANKSIZE * br - br * (br + 1) / 2 + bc;
                    const auto& blk = clusters[c].M[k];

                    for(int i = 0; i < 3; i++)
                        for(int j = 0; j < 3; j++)
                            fmt::format_to(std::back_inserter(buf),
                                           "{} {} {:.17g}\n",
                                           base + br * 3 + i + 1,
                                           base + bc * 3 + j + 1,
                                           static_cast<double>(blk(i, j)));
                }
            }
        }

        FILE* fp = std::fopen(path_str.c_str(), "w");
        if(!fp)
        {
            UIPC_WARN_WITH_LOCATION("MAS dump: open {} failed", path_str);
            return;
        }
        std::fwrite(buf.data(), 1, buf.size(), fp);
        std::fclose(fp);
        logger::info("MAS dump: wrote {}", path_str);
    };

    write_mtx(h_hess, "hess");
    write_mtx(h_inv, "inv");

    // Partition metadata as JSON
    {
        auto path =
            output_dir_path
            / fmt::format("mas_cluster_meta.f{}.n{}.json", frame, newton_iter);
        std::ofstream out(path);
        if(!out)
        {
            UIPC_WARN_WITH_LOCATION("MAS dump: open {} failed", path.string());
            return;
        }

        Json j;
        j["total_nodes"]        = m_total_nodes;
        j["total_map_nodes"]    = m_total_map_nodes;
        j["total_clusters"]     = m_total_num_clusters;
        j["num_cluster_blocks"] = nb;
        j["banksize"]           = BANKSIZE;
        j["sym_block_count"]    = SYM_BLOCK_COUNT;
        j["frame"]              = frame;
        j["newton_iter"]        = newton_iter;

        if(m_total_map_nodes > 0)
        {
            std::vector<int> h_part_to_real(static_cast<size_t>(m_total_map_nodes));
            part_to_real.view(0, m_total_map_nodes).copy_to(h_part_to_real.data());
            j["part_to_real"] = h_part_to_real;
        }

        // Hierarchy: level_sizes (start position .y, count .x per level)
        {
            std::vector<Int2> h_lvl(static_cast<size_t>(m_level_num + 1));
            level_sizes.view(0, m_level_num + 1).copy_to(h_lvl.data());
            Json jl = Json::array();
            for(int l = 0; l <= m_level_num; l++)
            {
                Json e;
                e["level"]  = l;
                e["count"]  = h_lvl[l].x;
                e["offset"] = h_lvl[l].y;
                jl.push_back(e);
            }
            j["levels"]    = jl;
            j["level_num"] = m_level_num;
        }

        // going_next: maps each "padded position in level X" -> "padded position in level X+1"
        if(m_total_num_clusters > 0)
        {
            std::vector<int> h_gn(static_cast<size_t>(m_total_num_clusters));
            going_next.view(0, m_total_num_clusters).copy_to(h_gn.data());
            j["going_next"] = h_gn;
        }

        // real_to_part: fine real index -> padded fine position (for restriction)
        if(m_total_nodes > 0)
        {
            std::vector<int> h_r2p(static_cast<size_t>(m_total_nodes));
            real_to_part.view(0, m_total_nodes).copy_to(h_r2p.data());
            j["real_to_part"] = h_r2p;
        }

        out << j.dump(4) << '\n';
        logger::info("MAS dump: wrote {}", path.string());
    }
}

}  // namespace uipc::backend::cuda
