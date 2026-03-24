#include <finite_element/mas_preconditioner_engine.h>
#include <muda/ext/eigen/atomic.h>
#include <muda/launch/launch.h>
#include <muda/launch/parallel_for.h>
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
MUDA_GENERIC unsigned int lanemask_lt(int lane_id)
{
    return (1U << lane_id) - 1;
}

// Symmetric upper-triangle index for a BANKSIZE x BANKSIZE block
MUDA_GENERIC int sym_index(int row, int col)
{
    return BANKSIZE * row - row * (row + 1) / 2 + col;
}

// Round n up to the nearest multiple of BANKSIZE
constexpr int bank_align(int n)
{
    return (n + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
}

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
    m_level_num = std::min(n_level, MAX_LEVELS);
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
    using namespace muda;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [neighbor_start = neighbor_starts.cviewer().name("neighbor_starts"),
                neighbor_num  = neighbor_nums.viewer().name("neighbor_nums"),
                neighbor_list = neighbor_lists.viewer().name("neighbor_lists"),
                fine_connect_mask = fine_connect_masks.viewer().name("fine_connect_masks"),
                part_to_real = part_to_real.cviewer().name("part_to_real"),
                real_to_part = real_to_part.cviewer().name("real_to_part")] __device__(int tid) mutable
               {
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
               });
}

// ---------------------------------------------------------------------------
// Prepare prefix sum at level 0 (uses shared memory)
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::prepare_prefix_sum_L0()
{
    using namespace muda;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [fine_connect_mask = fine_connect_masks.viewer().name("fine_connect_masks"),
             prefix_orig  = prefix_original.viewer().name("prefix_original"),
             part_to_real = part_to_real.cviewer().name("part_to_real"),
             N] __device__() mutable
            {
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
            });
}

// ---------------------------------------------------------------------------
// Build coarse level 1 from prefix sums
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_level1()
{
    using namespace muda;
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
    //   — when unpartitioned vertices exist (e.g. Empty constitution),
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

    Launch(num_blocks, block_size)
        .apply(
            [level_size = level_sizes.viewer().name("level_sizes"),
             coarse_table = coarse_space_tables.viewer().name("coarse_space_tables"),
             going_next_L0 = going_next.view(0, level1_begin).viewer().name("going_next_L0"),
             fine_connect_mask = fine_connect_masks.cviewer().name("fine_connect_masks"),
             prefix_sum_orig = prefix_sum_original.cviewer().name("prefix_sum_original"),
             prefix_orig  = prefix_original.cviewer().name("prefix_original"),
             part_to_real = part_to_real.cviewer().name("part_to_real"),
             N,
             level1_begin] __device__() mutable
            {
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
            });
}

// ---------------------------------------------------------------------------
// Build connectivity mask at level x (coarsened)
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_connect_mask_Lx(int level)
{
    using namespace muda;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [neighbor_start = neighbor_starts.cviewer().name("neighbor_starts"),
             neighbor_num   = neighbor_nums.viewer().name("neighbor_nums"),
             neighbor_list  = neighbor_lists.viewer().name("neighbor_lists"),
             coarse_table = coarse_space_tables.cviewer().name("coarse_space_tables"),
             next_connect_mask = next_connect_masks.viewer().name("next_connect_masks"),
             fine_connect_mask = fine_connect_masks.cviewer().name("fine_connect_masks"),
             part_to_real = part_to_real.cviewer().name("part_to_real"),
             level,
             vert_num = m_total_nodes,
             N] __device__() mutable
            {
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
            });
}

// ---------------------------------------------------------------------------
// Cluster connectivity at next level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::next_level_cluster(int level)
{
    using namespace muda;
    int N = m_h_level_size.x;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [next_connect_mask = next_connect_masks.viewer().name("next_connect_masks"),
             next_prefix = next_prefixes.viewer().name("next_prefixes"),
             N] __device__() mutable
            {
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
            });
}

// ---------------------------------------------------------------------------
// Prefix sum at level x
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::prefix_sum_Lx(int level)
{
    using namespace muda;
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

    Launch(num_blocks, block_size)
        .apply(
            [level_size_ptr = level_sizes.viewer().name("level_sizes"),
             next_prefix    = next_prefixes.cviewer().name("next_prefixes"),
             next_prefix_sum = next_prefix_sums.cviewer().name("next_prefix_sums"),
             next_connect_mask = next_connect_masks.viewer().name("next_connect_masks"),
             going_next_level = going_next.view(level_begin, level_region_size).viewer().name("going_next_Lx"),
             level,
             next_level_begin,
             N] __device__() mutable
            {
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
            });
}

// ---------------------------------------------------------------------------
// Compute next coarsening level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::compute_next_level(int level)
{
    using namespace muda;
    int N = m_total_nodes;
    if(N < 1)
        return;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [coarse_table = coarse_space_tables.viewer().name("coarse_space_tables"),
                next_connect_mask = next_connect_masks.cviewer().name("next_connect_masks"),
                level,
                N] __device__(int idx) mutable
               {
                   int next = coarse_table((level - 1) * N + idx);
                   if(next < 0 || next >= N)
                   {
                       coarse_table(level * N + idx) = -1;
                       return;
                   }
                   coarse_table(level * N + idx) = next_connect_mask(next);
               });
}

// ---------------------------------------------------------------------------
// Build per-node level traversal table
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::aggregation_kernel()
{
    using namespace muda;
    int N = m_total_nodes;
    if(N < 1 || m_total_num_clusters < 1)
        return;

    int level_num = m_level_num;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [coarse_table = coarse_tables.viewer().name("coarse_table"),
                going_next   = going_next.view(0, m_total_num_clusters).cviewer().name("going_next"),
                level_size   = level_sizes.cviewer().name("level_sizes"),
                level_num] __device__(int idx) mutable
               {
                   int        current_id = idx;
                   LevelTable ctable;

                   int first = going_next(current_id);
                   if(first >= 0)
                   {
                       MUDA_ASSERT(first >= level_size(1).y
                                       && first < level_size(2).y,
                                   "aggregation: going_next[%d]=%d not in level 1 [%d, %d)",
                                   current_id, first,
                                   level_size(1).y, level_size(2).y);

                       current_id      = first;
                       ctable.index[0] = first;

                       for(int l = 1; l < level_num - 1; l++)
                       {
                           int next = going_next(current_id);

                           MUDA_ASSERT(next >= 0,
                                       "aggregation: partitioned vertex %d has "
                                       "going_next=%d at level %d (expected >= 0)",
                                       idx, next, l + 1);

                           MUDA_ASSERT(next >= level_size(l + 1).y
                                           && next < level_size(l + 2).y,
                                       "aggregation: going_next[%d]=%d not in level %d [%d, %d)",
                                       current_id, next, l + 1,
                                       level_size(l + 1).y, level_size(l + 2).y);

                           current_id      = next;
                           ctable.index[l] = next;
                       }
                   }

                   coarse_table(idx) = ctable;
               });
}

// ============================================================================
// Phase 2: Assemble preconditioner
// ============================================================================

void MASPreconditionerEngine::set_preconditioner(muda::CBufferView<Eigen::Matrix3d> triplet_values,
                                                 muda::CBufferView<int>             row_ids,
                                                 muda::CBufferView<int>             col_ids,
                                                 muda::CBufferView<uint32_t>        indices,
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
void MASPreconditionerEngine::scatter_hessian_to_clusters(muda::CBufferView<Eigen::Matrix3d> triplet_values,
                                                          muda::CBufferView<int>             row_ids,
                                                          muda::CBufferView<int>             col_ids,
                                                          muda::CBufferView<uint32_t>        indices,
                                                          int dof_offset)
{
    using namespace muda;

    int triplet_num = static_cast<int>(indices.size());

    // --- Pass 1: Place each 3x3 block at the finest level where both
    //             row and col belong to the same cluster. ---

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(
            triplet_num,
            [offset          = dof_offset,
             level_num       = m_level_num,
             going_next      = going_next.view(0, m_total_num_clusters).cviewer().name("going_next"),
             level_size      = level_sizes.cviewer().name("level_sizes"),
             cluster_hess    = cluster_hessians.viewer().name("cluster_hessians"),
             real_to_part    = real_to_part.cviewer().name("real_to_part"),
             indices         = indices.cviewer().name("indices"),
             triplet_values  = triplet_values.cviewer().name("triplet_values"),
             row_ids         = row_ids.cviewer().name("row_ids"),
             col_ids         = col_ids.cviewer().name("col_ids"),
             total_nodes     = m_total_nodes] __device__(int I) mutable
            {
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
                        muda::eigen::atomic_add(cluster_hess(cluster_id).M[si], H);
                    }
                    else
                    {
                        Eigen::Matrix3d Ht = H.transpose();
                        int si = sym_index(vert_col % BANKSIZE, vert_row % BANKSIZE);
                        muda::eigen::atomic_add(cluster_hess(cluster_id).M[si], Ht);
                    }
                }
                else
                {
                    // Walk up levels until both nodes land in the same cluster
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

                        MUDA_ASSERT(vert_col >= level_size(level).y
                                        && vert_col < level_size(level + 1).y,
                                    "scatter P1: vert_col=%d not in level %d [%d, %d)",
                                    vert_col, level, level_size(level).y, level_size(level + 1).y);
                        MUDA_ASSERT(vert_row >= level_size(level).y
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
                            break;
                        }
                    }
                }
            });

    // --- Pass 2: Scatter fine-level cluster matrices to coarser levels
    //             using warp-reduction for contiguous partitions. ---

    int total_entries = m_total_map_nodes * BANKSIZE;
    if(total_entries < 1)
        return;
    int thread_num = BANKSIZE * BANKSIZE;
    int block_num  = (total_entries + thread_num - 1) / thread_num;

    ParallelFor(block_num, thread_num)
        .file_line(__FILE__, __LINE__)
        .apply(total_entries,
               [level_num    = m_level_num,
                going_next   = going_next.view(0, m_total_num_clusters).cviewer().name("going_next"),
                level_size   = level_sizes.cviewer().name("level_sizes"),
                cluster_hess = cluster_hessians.viewer().name("cluster_hessians"),
                part_to_real = part_to_real.cviewer().name("part_to_real"),
                fine_connect = fine_connect_masks.cviewer().name("fine_connect_masks"),
                prefix_orig  = prefix_original.cviewer().name("prefix_original"),
                map_nodes    = m_total_map_nodes] __device__(int idx) mutable
               {
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

                   if(rdx < 0 || cdx < 0)
                       return;

                   if(prefix == 1)
                   {
                       using WarpReduceD = cub::WarpReduce<double>;
                       __shared__
                           typename WarpReduceD::TempStorage temp_reduce_d[BANKSIZE * BANKSIZE / 32];
                       int hw_warp = threadIdx.x / 32;

                       for(int i = 0; i < 3; i++)
                           for(int j = 0; j < 3; j++)
                               mat3(i, j) =
                                   WarpReduceD(temp_reduce_d[hw_warp]).Sum(mat3(i, j));

                       if((threadIdx.x & 0x1f) == 0)
                       {
                           int level   = 0;
                           int next_id = going_next(rdx);
                           if(next_id < 0)
                               return;

                           while(level < level_num - 1)
                           {
                               level++;

                               MUDA_ASSERT(next_id >= level_size(level).y
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
                               if(next_id < 0)
                                   return;
                           }
                       }
                   }
                   else
                   {
                       int level = 1;
                       while(level <= level_num - 1)
                       {
                           rdx = going_next(rdx);
                           cdx = going_next(cdx);
                           if(rdx < 0 || cdx < 0)
                               return;

                           MUDA_ASSERT(rdx >= level_size(level).y
                                           && rdx < level_size(level + 1).y,
                                       "scatter P2: rdx=%d not in level %d [%d, %d)",
                                       rdx, level, level_size(level).y, level_size(level + 1).y);
                           MUDA_ASSERT(cdx >= level_size(level).y
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
               });
}

// ---------------------------------------------------------------------------
// Gauss-Jordan inversion of each 48x48 cluster matrix
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::invert_cluster_matrices()
{
    using namespace muda;
    int total_threads = m_total_num_clusters * 3;  // 48 threads per cluster
    if(total_threads < 1)
        return;

    int block_size = 32 * 3;  // 96 threads = 2 clusters per block
    int num_blocks = (total_threads + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [cluster_inv  = cluster_inverses.viewer().name("cluster_inverses"),
             cluster_hess = cluster_hessians.cviewer().name("cluster_hessians"),
             total_threads] __device__() mutable
            {
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
            });
}

// ============================================================================
// Phase 3: Apply preconditioning  z = M^{-1} r
// ============================================================================

// ---------------------------------------------------------------------------
// Restrict: accumulate residual R from fine to all coarser levels
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::build_multi_level_R(muda::CDenseVectorView<Float> R,
                                                  muda::CVarView<IndexT> converged)
{
    using namespace muda;
    int N = m_total_map_nodes;
    if(N < 1)
        return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [R_view       = R.cviewer().name("R"),
             multi_lr     = multi_level_R.viewer().name("multi_level_R"),
             going_next   = going_next.view(0, m_total_num_clusters).cviewer().name("going_next"),
             level_size   = level_sizes.cviewer().name("level_sizes"),
             prefix_orig  = prefix_original.cviewer().name("prefix_original"),
             fine_conn    = fine_connect_masks.cviewer().name("fine_connect_masks"),
             part_to_real = part_to_real.cviewer().name("part_to_real"),
             converged    = converged.cviewer().name("converged"),
             level_num    = m_level_num,
             N] __device__() mutable
            {
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

                            MUDA_ASSERT(cur >= level_size(l + 1).y
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

                            MUDA_ASSERT(cur >= level_size(l + 1).y
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
            });
}

// ---------------------------------------------------------------------------
// Local solve: Z = cluster_inverse * R at each level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::schwarz_local_solve(muda::CVarView<IndexT> converged)
{
    using namespace muda;
    int N = m_total_num_clusters * BANKSIZE;  // one thread per (cluster, node-pair)
    if(N < 1)
        return;

    int block_size = BANKSIZE * BANKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [cluster_inv = cluster_inverses.cviewer().name("cluster_inverses"),
             multi_lr    = multi_level_R.cviewer().name("multi_level_R"),
             multi_lz    = multi_level_Z.viewer().name("multi_level_Z"),
             converged   = converged.cviewer().name("converged"),
             N] __device__() mutable
            {
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
            });
}

// ---------------------------------------------------------------------------
// Prolongate: sum Z contributions from all levels for each fine node
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::collect_final_Z(muda::DenseVectorView<Float> Z,
                                              muda::CVarView<IndexT> converged)
{
    using namespace muda;
    int N = m_total_nodes;
    if(N < 1)
        return;

    int level_num = m_level_num;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [Z_view       = Z.viewer().name("Z"),
                multi_lz     = multi_level_Z.cviewer().name("multi_level_Z"),
                coarse_table = coarse_tables.cviewer().name("coarse_table"),
                real_to_part = real_to_part.cviewer().name("real_to_part"),
                converged    = converged.cviewer().name("converged"),
                level_num] __device__(int idx) mutable
               {
                   if(*converged != 0)
                       return;
                   int rdx = real_to_part(idx);

                   // Skip unpartitioned vertices (handled by diagonal fallback)
                   if(rdx < 0)
                       return;

                   // Start with the fine-level solution
                   float3 cz = multi_lz(rdx);

                   // Add contributions from all coarser levels
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
               });
}

// ============================================================================
// Apply: full preconditioning pipeline  z = M^{-1} r
// ============================================================================

void MASPreconditionerEngine::apply(muda::CDenseVectorView<Float> r,
                                    muda::DenseVectorView<Float>  z,
                                    muda::CVarView<IndexT>        converged)
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

void MASPreconditionerEngine::dump_cluster_matrices_debug(const std::filesystem::path& output_dir,
                                                          SizeT frame,
                                                          SizeT newton_iter)
{
    if(!m_initialized || cluster_hessians.size() == 0)
        return;

    muda::wait_device();

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
            output_dir
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
            output_dir
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

        out << j.dump(4) << '\n';
        logger::info("MAS dump: wrote {}", path.string());
    }
}

}  // namespace uipc::backend::cuda
