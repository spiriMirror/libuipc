#include <finite_element/mas_preconditioner_engine.h>
#include <muda/launch/launch.h>
#include <muda/launch/parallel_for.h>
#include <thrust/device_ptr.h>
#include <thrust/scan.h>

namespace uipc::backend::cuda
{
// ============================================================================
// Local constants & type aliases
// ============================================================================
static constexpr int BANKSIZE          = MASPreconditionerEngine::BANKSIZE;
static constexpr int DEFAULT_BLOCKSIZE = MASPreconditionerEngine::DEFAULT_BLOCKSIZE;
static constexpr int DEFAULT_WARPNUM   = MASPreconditionerEngine::DEFAULT_WARPNUM;
static constexpr int SYM_BLOCK_COUNT   = MASPreconditionerEngine::SYM_BLOCK_COUNT;

using ClusterMatrixSym  = MASPreconditionerEngine::ClusterMatrixSym;
using ClusterMatrixSymF = MASPreconditionerEngine::ClusterMatrixSymF;
using LevelTable        = MASPreconditionerEngine::LevelTable;
using Int2              = MASPreconditionerEngine::Int2;

// Device helper: bitmask with bits [0, lane_id) set
MUDA_DEVICE unsigned int lanemask_lt(int lane_id)
{
    return (1U << lane_id) - 1;
}

// Symmetric upper-triangle index for a BANKSIZE x BANKSIZE block
MUDA_DEVICE int sym_index(int row, int col)
{
    return BANKSIZE * row - row * (row + 1) / 2 + col;
}

// ============================================================================
// Phase 1: Initialization
// ============================================================================

void MASPreconditionerEngine::compute_num_levels(int vert_num)
{
    int n_level  = 1;
    int level_sz = (vert_num + BANKSIZE - 1) / BANKSIZE * BANKSIZE;

    while(level_sz > BANKSIZE)
    {
        level_sz /= BANKSIZE;
        n_level++;
        level_sz = (level_sz + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
    }
    n_level++;
    m_level_num = std::min(n_level, MAX_LEVELS);
}

void MASPreconditionerEngine::init_neighbor(
    int                              vert_num,
    int                              total_neighbor_num,
    int                              part_map_size,
    const std::vector<unsigned int>& h_neighbor_list,
    const std::vector<unsigned int>& h_neighbor_start,
    const std::vector<unsigned int>& h_neighbor_num,
    const std::vector<int>&          h_part_to_real,
    const std::vector<int>&          h_real_to_part)
{
    if(vert_num < 1)
        return;

    int max_nodes = std::max(part_map_size, vert_num);
    compute_num_levels(max_nodes);

    m_total_map_nodes = part_map_size;
    m_total_nodes     = vert_num;

    // Hierarchy buffers
    dense_level.resize(vert_num);
    real_to_part.resize(vert_num);
    coarse_tables.resize(vert_num);
    coarse_space_tables.resize(vert_num * m_level_num);
    level_sizes.resize(m_level_num + 1);
    going_next.resize(vert_num * m_level_num);
    prefix_original.resize(vert_num);
    next_prefixes.resize(vert_num);
    next_prefix_sums.resize(vert_num);
    prefix_sum_original.resize(vert_num);
    fine_connect_masks.resize(vert_num);
    next_connect_masks.resize(vert_num);

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

    build_connect_mask_L0();

    // Contact-aware: inject collision connectivity at fine level
    if(m_collision_num > 0)
        build_collision_connection(fine_connect_masks.data(), nullptr, -1, m_collision_num);

    prepare_prefix_sum_L0();
    build_level1();

    for(int level = 1; level < m_level_num; level++)
    {
        next_connect_masks.fill(0u);
        build_connect_mask_Lx(level);

        // Contact-aware: inject collision connectivity at coarser levels
        if(m_collision_num > 0)
            build_collision_connection(next_connect_masks.data(),
                                      coarse_space_tables.data(),
                                      level,
                                      m_collision_num);

        // Copy level size to host for next-level sizing
        cudaMemcpy(&m_h_level_size,
                   level_sizes.data() + level,
                   sizeof(Int2),
                   cudaMemcpyDeviceToHost);

        next_level_cluster(level);
        prefix_sum_Lx(level);
        compute_next_level(level);
    }

    cudaMemcpy(&m_h_level_size,
               level_sizes.data() + m_level_num,
               sizeof(Int2),
               cudaMemcpyDeviceToHost);

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
               [neighbor_start    = neighbor_starts.data(),
                neighbor_num      = neighbor_nums.data(),
                neighbor_list     = neighbor_lists.data(),
                fine_connect_mask = fine_connect_masks.data(),
                part_to_real      = part_to_real.cviewer().name("part_to_real"),
                real_to_part      = real_to_part.cviewer().name("real_to_part")]
               __device__(int tid) mutable
               {
                   int bank_id = tid / BANKSIZE;
                   int lane_id = tid % BANKSIZE;
                   int idx     = part_to_real(tid);

                   if(idx < 0)
                       return;

                   int          num_nbr     = neighbor_num[idx];
                   unsigned int connect_msk = (1U << lane_id);
                   int          nk          = 0;
                   int          start_id    = neighbor_start[idx];

                   for(int i = 0; i < num_nbr; i++)
                   {
                       int nbr_id        = neighbor_list[start_id + i];
                       int nbr_bank_id   = real_to_part(nbr_id) / BANKSIZE;
                       if(bank_id == nbr_bank_id)
                       {
                           unsigned int nbr_lane = real_to_part(nbr_id) % BANKSIZE;
                           connect_msk |= (1U << nbr_lane);
                       }
                       else
                       {
                           neighbor_list[start_id + nk] = nbr_id;
                           nk++;
                       }
                   }
                   neighbor_num[idx]      = nk;
                   fine_connect_mask[idx]  = connect_msk;
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
            [fine_connect_mask = fine_connect_masks.data(),
             prefix_orig       = prefix_original.data(),
             part_to_real      = part_to_real.cviewer().name("part_to_real"),
             N] __device__() mutable
            {
                int tid = blockIdx.x * blockDim.x + threadIdx.x;
                if(tid >= N) return;

                int warp_id       = tid / BANKSIZE;
                int local_warp_id = threadIdx.x / BANKSIZE;
                int lane_id       = tid % BANKSIZE;
                int idx           = part_to_real(tid);

                __shared__ unsigned int cache_mask[DEFAULT_BLOCKSIZE];
                __shared__ int          prefix_sum[DEFAULT_WARPNUM];

                if(idx >= 0)
                {
                    unsigned int connect_msk = fine_connect_mask[idx];
                    if(lane_id == 0)
                        prefix_sum[local_warp_id] = 0;
                    cache_mask[threadIdx.x] = connect_msk;
                    unsigned int visited = (1U << lane_id);

                    // Flood-fill connectivity within the bank via shared memory
                    while(connect_msk != ~0U)
                    {
                        unsigned int todo = visited ^ connect_msk;
                        if(!todo) break;
                        unsigned int next_visit = __ffs(todo) - 1;
                        visited |= (1U << next_visit);
                        connect_msk |= cache_mask[next_visit + local_warp_id * BANKSIZE];
                    }

                    fine_connect_mask[idx] = connect_msk;

                    unsigned int elected_prefix = __popc(connect_msk & lanemask_lt(lane_id));
                    if(elected_prefix == 0)
                        atomicAdd(prefix_sum + local_warp_id, 1);
                    if(lane_id == 0)
                        prefix_orig[warp_id] = prefix_sum[local_warp_id];
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

    Launch(num_blocks, block_size)
        .apply(
            [level_size        = level_sizes.data(),
             coarse_table      = coarse_space_tables.data(),
             going_next_ptr    = going_next.data(),
             fine_connect_mask = fine_connect_masks.data(),
             prefix_sum_orig   = prefix_sum_original.data(),
             prefix_orig       = prefix_original.data(),
             part_to_real      = part_to_real.cviewer().name("part_to_real"),
             N] __device__() mutable
            {
                int tid = blockIdx.x * blockDim.x + threadIdx.x;
                if(tid >= N) return;

                int warp_id       = tid / BANKSIZE;
                int local_warp_id = threadIdx.x / BANKSIZE;
                int lane_id       = tid % BANKSIZE;

                __shared__ unsigned int elected_mask[BANKSIZE];
                __shared__ unsigned int lane_prefix[BANKSIZE * BANKSIZE];

                if(lane_id == 0)
                    elected_mask[local_warp_id] = 0;

                if(tid == N - 1)
                {
                    level_size[1].x = prefix_sum_orig[warp_id] + prefix_orig[warp_id];
                    level_size[1].y = (N + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
                }

                int idx = part_to_real(tid);
                if(idx >= 0)
                {
                    unsigned int conn_msk       = fine_connect_mask[idx];
                    unsigned int elected_prefix = __popc(conn_msk & lanemask_lt(lane_id));
                    if(elected_prefix == 0)
                        atomicOr(elected_mask + local_warp_id, (1U << lane_id));

                    lane_prefix[threadIdx.x] =
                        __popc(elected_mask[local_warp_id] & lanemask_lt(lane_id));
                    lane_prefix[threadIdx.x] += prefix_sum_orig[warp_id];

                    unsigned int elected_lane   = __ffs(conn_msk) - 1;
                    unsigned int the_lane_prefix =
                        lane_prefix[elected_lane + BANKSIZE * local_warp_id];

                    coarse_table[idx]    = the_lane_prefix;
                    going_next_ptr[idx]  = the_lane_prefix
                        + (N + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
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
    if(N < 1) return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [neighbor_start    = neighbor_starts.data(),
             neighbor_num      = neighbor_nums.data(),
             neighbor_list     = neighbor_lists.data(),
             coarse_table      = coarse_space_tables.data(),
             next_connect_mask = next_connect_masks.data(),
             fine_connect_mask = fine_connect_masks.data(),
             part_to_real      = part_to_real.cviewer().name("part_to_real"),
             level, vert_num = m_total_nodes, N] __device__() mutable
            {
                int tid = blockIdx.x * blockDim.x + threadIdx.x;
                if(tid >= N) return;

                int local_warp_id = threadIdx.x / BANKSIZE;
                int lane_id       = tid % BANKSIZE;

                __shared__ int cache_msk[DEFAULT_BLOCKSIZE];

                int idx = part_to_real(tid);
                if(idx < 0) return;

                unsigned int prefix_msk = fine_connect_mask[idx];
                unsigned int conn_msk   = 0;
                unsigned int coarse_idx = coarse_table[(level - 1) * vert_num + idx];
                int kn       = neighbor_num[idx];
                int nk       = 0;
                int start_id = neighbor_start[idx];

                for(int i = 0; i < kn; i++)
                {
                    unsigned int connect       = neighbor_list[start_id + i];
                    unsigned int coarse_connect = coarse_table[(level - 1) * vert_num + connect];
                    if(coarse_idx / BANKSIZE == coarse_connect / BANKSIZE)
                    {
                        conn_msk |= (1U << (coarse_connect % BANKSIZE));
                    }
                    else
                    {
                        neighbor_list[start_id + nk] = connect;
                        nk++;
                    }
                }
                neighbor_num[idx] = nk;
                cache_msk[threadIdx.x] = 0;

                if(__popc(prefix_msk) == BANKSIZE)
                {
                    atomicOr(cache_msk + local_warp_id * BANKSIZE, (int)conn_msk);
                    conn_msk = (unsigned int)cache_msk[local_warp_id * BANKSIZE];
                }
                else
                {
                    unsigned int elected_lane = __ffs(prefix_msk) - 1;
                    if(conn_msk)
                        atomicOr(cache_msk + local_warp_id * BANKSIZE + elected_lane, (int)conn_msk);
                    conn_msk = (unsigned int)cache_msk[local_warp_id * BANKSIZE + elected_lane];
                }

                unsigned int elected_prefix = __popc(prefix_msk & lanemask_lt(lane_id));
                if(conn_msk && elected_prefix == 0)
                    atomicOr(next_connect_mask + coarse_idx, conn_msk);
            });
}

// ---------------------------------------------------------------------------
// Cluster connectivity at next level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::next_level_cluster(int level)
{
    using namespace muda;
    int N = m_h_level_size.x;
    if(N < 1) return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [next_connect_mask = next_connect_masks.data(),
             next_prefix       = next_prefixes.data(),
             N] __device__() mutable
            {
                int idx = blockIdx.x * blockDim.x + threadIdx.x;
                if(idx >= N) return;

                int local_warp_id = threadIdx.x / BANKSIZE;
                int lane_id       = idx % BANKSIZE;

                __shared__ int          prefix_sum_s[DEFAULT_WARPNUM];
                __shared__ unsigned int cached_msk[DEFAULT_BLOCKSIZE];

                if(lane_id == 0) prefix_sum_s[local_warp_id] = 0;

                unsigned int conn_msk = (1U << lane_id) | next_connect_mask[idx];
                cached_msk[threadIdx.x] = conn_msk;
                unsigned int visited = (1U << lane_id);

                while(true)
                {
                    unsigned int todo = visited ^ conn_msk;
                    if(!todo) break;
                    unsigned int next_visit = __ffs(todo) - 1;
                    visited |= (1U << next_visit);
                    conn_msk |= cached_msk[next_visit + local_warp_id * BANKSIZE];
                }

                next_connect_mask[idx] = conn_msk;
                unsigned int elected_prefix = __popc(conn_msk & lanemask_lt(lane_id));
                if(elected_prefix == 0)
                    atomicAdd(prefix_sum_s + local_warp_id, 1);
                if(lane_id == 0)
                    next_prefix[idx / BANKSIZE] = prefix_sum_s[local_warp_id];
            });
}

// ---------------------------------------------------------------------------
// Prefix sum at level x
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::prefix_sum_Lx(int level)
{
    using namespace muda;
    int N = m_h_level_size.x;
    if(N < 1) return;

    int level_begin = m_h_level_size.y;
    int block_size  = BANKSIZE * BANKSIZE;
    int num_blocks  = (N + block_size - 1) / block_size;
    int warp_num    = (N + BANKSIZE - 1) / BANKSIZE;

    thrust::exclusive_scan(
        thrust::device_ptr<unsigned int>(next_prefixes.data()),
        thrust::device_ptr<unsigned int>(next_prefixes.data()) + warp_num,
        thrust::device_ptr<unsigned int>(next_prefix_sums.data()));

    Launch(num_blocks, block_size)
        .apply(
            [level_size_ptr    = level_sizes.data(),
             next_prefix       = next_prefixes.data(),
             next_prefix_sum   = next_prefix_sums.data(),
             next_connect_mask = next_connect_masks.data(),
             going_next_ptr    = going_next.data(),
             level, level_begin, N] __device__() mutable
            {
                int idx = blockIdx.x * blockDim.x + threadIdx.x;
                if(idx >= N) return;

                int warp_id       = idx / BANKSIZE;
                int local_warp_id = threadIdx.x / BANKSIZE;
                int lane_id       = idx % BANKSIZE;

                __shared__ unsigned int elected_mask[BANKSIZE];
                __shared__ unsigned int lane_prefix[BANKSIZE * BANKSIZE];

                if(lane_id == 0)
                    elected_mask[local_warp_id] = 0;

                if(idx == N - 1)
                {
                    level_size_ptr[level + 1].x =
                        next_prefix_sum[warp_id] + next_prefix[warp_id];
                    level_size_ptr[level + 1].y =
                        level_begin + (N + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
                }

                unsigned int conn_msk       = next_connect_mask[idx];
                unsigned int elected_prefix = __popc(conn_msk & lanemask_lt(lane_id));
                if(elected_prefix == 0)
                    atomicOr(elected_mask + local_warp_id, (1U << lane_id));

                lane_prefix[threadIdx.x] =
                    __popc(elected_mask[local_warp_id] & lanemask_lt(lane_id));
                lane_prefix[threadIdx.x] += next_prefix_sum[warp_id];

                unsigned int elected_lane    = __ffs(conn_msk) - 1;
                unsigned int the_lane_prefix =
                    lane_prefix[elected_lane + BANKSIZE * local_warp_id];

                next_connect_mask[idx] = the_lane_prefix;
                going_next_ptr[idx + level_begin] =
                    the_lane_prefix + level_begin
                    + (N + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            });
}

// ---------------------------------------------------------------------------
// Compute next coarsening level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::compute_next_level(int level)
{
    using namespace muda;
    int N = m_total_nodes;
    if(N < 1) return;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [coarse_table      = coarse_space_tables.data(),
                next_connect_mask = next_connect_masks.data(),
                level, N] __device__(int idx) mutable
               {
                   int next = coarse_table[(level - 1) * N + idx];
                   coarse_table[level * N + idx] = next_connect_mask[next];
               });
}

// ---------------------------------------------------------------------------
// Build per-node level traversal table
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::aggregation_kernel()
{
    using namespace muda;
    int N = m_total_nodes;
    if(N < 1) return;

    int level_num = m_level_num;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [dense_level   = dense_level.data(),
                coarse_table  = coarse_tables.viewer().name("coarse_table"),
                going_next    = going_next.cviewer().name("going_next"),
                level_num] __device__(int idx) mutable
               {
                   int        current_id = idx;
                   LevelTable ctable;
                   for(int l = 0; l < level_num - 1; l++)
                   {
                       int next        = going_next(current_id);
                       current_id      = next;
                       ctable.index[l] = next;
                   }
                   coarse_table(idx) = ctable;
               });
}

// ============================================================================
// Collision connectivity (contact-aware MAS)
// ============================================================================

void MASPreconditionerEngine::set_collision_pairs(const int* d_collision_pairs,
                                                   int        num_pairs,
                                                   int        node_offset)
{
    m_collision_pairs       = d_collision_pairs;
    m_collision_num         = num_pairs;
    m_collision_node_offset = node_offset;
}

void MASPreconditionerEngine::build_collision_connection(
    unsigned int* connection_mask,
    const int*    coarse_table,   // nullptr for L0
    int           level,
    int           cp_num)
{
    using namespace muda;
    if(cp_num < 1) return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (cp_num + block_size - 1) / block_size;

    // Each collision pair has 2-4 vertices (packed as int4).
    // For each pair of vertices that map to the same bank (cluster),
    // set the connectivity bit so they're grouped together.
    Launch(num_blocks, block_size)
        .apply(
            [connection_mask,
             coarse_table,
             collision_pairs = m_collision_pairs,
             real_to_part    = real_to_part.data(),
             node_offset     = m_collision_node_offset,
             vert_num        = m_total_nodes,
             level,
             cp_num] __device__() mutable
            {
                int idx = blockIdx.x * blockDim.x + threadIdx.x;
                if(idx >= cp_num) return;

                // Read the collision pair (int4-packed: up to 4 vertex indices)
                const int* pair = collision_pairs + idx * 4;
                int verts[4] = {pair[0], pair[1], pair[2], pair[3]};

                // Map each vertex to partition space
                int cp_vid[4];
                for(int i = 0; i < 4; i++)
                {
                    int v = verts[i];
                    if(v >= 0)
                    {
                        v -= node_offset;
                        if(v >= 0 && v < vert_num)
                        {
                            if(coarse_table)
                                cp_vid[i] = coarse_table[(level - 1) * vert_num + v];
                            else
                                cp_vid[i] = real_to_part[v];
                        }
                        else
                            cp_vid[i] = -1;
                    }
                    else
                        cp_vid[i] = -1;
                }

                // For each pair of vertices in the same bank, set connectivity
                for(int i = 0; i < 4; i++)
                {
                    for(int j = i + 1; j < 4; j++)
                    {
                        int a = cp_vid[i];
                        int b = cp_vid[j];

                        if(a < 0 || b < 0 || a == b) continue;

                        if(a / BANKSIZE == b / BANKSIZE)
                        {
                            unsigned int mask_a = (1U << (b % BANKSIZE));
                            unsigned int mask_b = (1U << (a % BANKSIZE));

                            if(coarse_table)
                            {
                                atomicOr(connection_mask + a, mask_a);
                                atomicOr(connection_mask + b, mask_b);
                            }
                            else
                            {
                                // For L0: connection_mask is indexed by real vertex
                                int real_a = verts[i] - node_offset;
                                int real_b = verts[j] - node_offset;
                                if(real_a >= 0 && real_a < vert_num)
                                    atomicOr(connection_mask + real_a, mask_a);
                                if(real_b >= 0 && real_b < vert_num)
                                    atomicOr(connection_mask + real_b, mask_b);
                            }
                        }
                    }
                }
            });
}

// ============================================================================
// Phase 2: Assemble preconditioner
// ============================================================================

void MASPreconditionerEngine::set_preconditioner(
    const Eigen::Matrix3d* d_triplet_values,
    const int*             d_row_ids,
    const int*             d_col_ids,
    const uint32_t*        d_indices,
    int                    dof_offset,
    int                    triplet_num,
    int                    cp_num)
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

    // 4. Clear cluster Hessian matrices
    // NOTE: cudaMemset is used here because muda::DeviceBuffer::fill
    //       does not reliably zero-initialize large structs containing
    //       Eigen matrices due to force_trivially_constructible.
    cudaMemset(cluster_hessians.data(), 0,
               num_cluster_blocks * sizeof(ClusterMatrixSym));

    // 5. Scatter BCOO Hessian blocks into cluster matrices
    scatter_hessian_to_clusters(d_triplet_values, d_row_ids, d_col_ids,
                                d_indices, dof_offset, triplet_num);

    // 6. Invert each cluster matrix (Gauss-Jordan)
    invert_cluster_matrices();
}

// ---------------------------------------------------------------------------
// Scatter BCOO Hessian entries into cluster-level dense matrices
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::scatter_hessian_to_clusters(
    const Eigen::Matrix3d* d_triplet_values,
    const int*             d_row_ids,
    const int*             d_col_ids,
    const uint32_t*        d_indices,
    int                    dof_offset,
    int                    triplet_num)
{
    using namespace muda;

    // --- Pass 1: Place each 3x3 block at the finest level where both
    //             row and col belong to the same cluster. ---

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(
            triplet_num,
            [offset         = dof_offset,
             level_num      = m_level_num,
             going_next     = going_next.data(),
             cluster_hess   = cluster_hessians.data(),
             real_to_part   = real_to_part.data(),
             indices        = d_indices,
             triplet_values = d_triplet_values,
             row_ids        = d_row_ids,
             col_ids        = d_col_ids,
             total_nodes    = m_total_nodes] __device__(int I) mutable
            {
                int  index    = indices[I];
                int  row_real = row_ids[index] - offset;
                int  col_real = col_ids[index] - offset;
                auto H        = triplet_values[index];

                // Skip triplets outside FEM DOF range
                if(row_real < 0 || row_real >= total_nodes
                   || col_real < 0 || col_real >= total_nodes)
                    return;

                int vert_col = real_to_part[col_real];
                int vert_row = real_to_part[row_real];

                if(vert_col / BANKSIZE == vert_row / BANKSIZE)
                {
                    int cluster_id = vert_col / BANKSIZE;
                    if(vert_col >= vert_row)
                    {
                        int si = sym_index(vert_row % BANKSIZE, vert_col % BANKSIZE);
                        cluster_hess[cluster_id].M[si] = H;
                    }
                    else
                    {
                        // Partition reorder flipped order; store transpose
                        int si = sym_index(vert_col % BANKSIZE, vert_row % BANKSIZE);
                        cluster_hess[cluster_id].M[si] = H.transpose();
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
                            vert_col = going_next[col_real];
                            vert_row = going_next[row_real];
                        }
                        else
                        {
                            vert_col = going_next[vert_col];
                            vert_row = going_next[vert_row];
                        }

                        if(vert_col / BANKSIZE == vert_row / BANKSIZE)
                        {
                            int cluster_id = vert_col / BANKSIZE;
                            if(vert_col >= vert_row)
                            {
                                int si = sym_index(vert_row % BANKSIZE, vert_col % BANKSIZE);
                                for(int i = 0; i < 3; i++)
                                    for(int j = 0; j < 3; j++)
                                    {
                                        atomicAdd(&(cluster_hess[cluster_id].M[si](i, j)), H(i, j));
                                        if(vert_col == vert_row)
                                            atomicAdd(&(cluster_hess[cluster_id].M[si](i, j)), H(j, i));
                                    }
                            }
                            else
                            {
                                int si = sym_index(vert_col % BANKSIZE, vert_row % BANKSIZE);
                                for(int i = 0; i < 3; i++)
                                    for(int j = 0; j < 3; j++)
                                        atomicAdd(&(cluster_hess[cluster_id].M[si](i, j)), H(j, i));
                            }
                            break;  // done for this triplet
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
        .apply(
            total_entries,
            [level_num       = m_level_num,
             going_next      = going_next.data(),
             cluster_hess    = cluster_hessians.data(),
             part_to_real    = part_to_real.data(),
             fine_connect    = fine_connect_masks.data(),
             prefix_orig     = prefix_original.data()] __device__(int idx) mutable
            {
                int cluster_stride = BANKSIZE * BANKSIZE;
                int cluster_id     = idx / cluster_stride;
                int local_row      = (idx % cluster_stride) / BANKSIZE;
                int local_col      = (idx % cluster_stride) % BANKSIZE;

                int global_row = cluster_id * BANKSIZE + local_row;
                int global_col = cluster_id * BANKSIZE + local_col;

                int rdx = part_to_real[global_row];
                int cdx = part_to_real[global_col];

                __shared__ int prefix;
                if(threadIdx.x == 0)
                    prefix = prefix_orig[cluster_id];
                __syncthreads();

                // Read the 3x3 block from the symmetric cluster matrix
                Eigen::Matrix3d mat3;
                if(local_col >= local_row)
                {
                    int si = sym_index(local_row, local_col);
                    mat3   = cluster_hess[cluster_id].M[si];
                }
                else
                {
                    int si = sym_index(local_col, local_row);
                    mat3   = cluster_hess[cluster_id].M[si].transpose();
                }

                if(rdx < 0 || cdx < 0)
                    return;

                if(prefix == 1)
                {
                    // Contiguous partition: warp-reduce the matrix
                    int  warp_id    = threadIdx.x & 0x1f;
                    bool is_boundary = (warp_id == 0) || (rdx < 0) || (cdx < 0);
                    unsigned int mark = __ballot_sync(0xffffffff, is_boundary);
                    mark = __brev(mark);
                    int clz_len = __clz(mark << (warp_id + 1));
                    unsigned int interval = min(clz_len, 31 - warp_id);

                    for(int iter = 1; iter < 32; iter <<= 1)
                    {
                        Eigen::Matrix3d tmp;
                        for(int i = 0; i < 3; i++)
                            for(int j = 0; j < 3; j++)
                                tmp(i, j) = __shfl_down_sync(0xffffffff, mat3(i, j), iter);
                        if(interval >= (unsigned int)iter)
                            mat3 = mat3 + tmp;
                    }

                    if(is_boundary)
                    {
                        int level   = 0;
                        int next_id = going_next[rdx];
                        while(level < level_num - 1)
                        {
                            level++;
                            int cid     = next_id / BANKSIZE;
                            int bv      = next_id % BANKSIZE;
                            int si      = sym_index(bv, bv);
                            for(int i = 0; i < 3; i++)
                                for(int j = 0; j < 3; j++)
                                    atomicAdd(&(cluster_hess[cid].M[si](i, j)), mat3(i, j));
                            next_id = going_next[next_id];
                        }
                    }
                }
                else
                {
                    // Non-contiguous: per-entry scatter to coarser levels
                    int level = 0;
                    while(level < level_num - 1)
                    {
                        level++;
                        rdx      = going_next[rdx];
                        cdx      = going_next[cdx];
                        int cid  = cdx / BANKSIZE;
                        if(rdx / BANKSIZE == cdx / BANKSIZE)
                        {
                            if(cdx >= rdx)
                            {
                                int si = sym_index(rdx % BANKSIZE, cdx % BANKSIZE);
                                for(int i = 0; i < 3; i++)
                                    for(int j = 0; j < 3; j++)
                                        atomicAdd(&(cluster_hess[cid].M[si](i, j)), mat3(i, j));
                            }
                        }
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
            [cluster_inv  = cluster_inverses.data(),
             cluster_hess = cluster_hessians.data(),
             total_threads] __device__() mutable
            {
                int idx = blockIdx.x * blockDim.x + threadIdx.x;
                if(idx >= total_threads) return;

                constexpr int MAT_DIM = BANKSIZE * 3;  // 48

                int mat_id       = idx / MAT_DIM;
                int col          = idx % MAT_DIM;  // this thread owns one column
                int block_mat_id = threadIdx.x / MAT_DIM;

                // Load the full symmetric matrix into shared memory
                __shared__ double s_mat[32 / BANKSIZE][MAT_DIM][MAT_DIM];
                __shared__ double s_col[32 / BANKSIZE][MAT_DIM];

                for(int row = 0; row < MAT_DIM; row++)
                {
                    int node_row = row / 3;
                    int node_col = col / 3;
                    if(node_col >= node_row)
                    {
                        int si = sym_index(node_row, node_col);
                        s_mat[block_mat_id][row][col] =
                            cluster_hess[mat_id].M[si](row % 3, col % 3);
                    }
                    else
                    {
                        int si = sym_index(node_col, node_row);
                        s_mat[block_mat_id][row][col] =
                            cluster_hess[mat_id].M[si](col % 3, row % 3);
                    }
                    // Regularize: zero diagonal → identity (for padding nodes)
                    if(row == col && s_mat[block_mat_id][row][col] == 0.0)
                        s_mat[block_mat_id][row][col] = 1.0;
                }

                // Gauss-Jordan elimination (in-place inversion)
                for(int pivot = 0; pivot < MAT_DIM; pivot++)
                {
                    __syncthreads();
                    double pivot_val = s_mat[block_mat_id][pivot][pivot];
                    s_col[block_mat_id][col] = s_mat[block_mat_id][col][pivot];
                    __syncthreads();

                    s_mat[block_mat_id][col == pivot ? col : col][pivot] =
                        (col == pivot) ? 1.0 : 0.0;

                    __syncthreads();
                    s_mat[block_mat_id][pivot][col] /= pivot_val;
                    __syncthreads();

                    for(int row = 0; row < MAT_DIM; row++)
                    {
                        if(row != pivot)
                        {
                            double factor = -s_col[block_mat_id][row];
                            __syncthreads();
                            s_mat[block_mat_id][row][col] +=
                                factor * s_mat[block_mat_id][pivot][col];
                        }
                    }
                }
                __syncthreads();

                // Symmetrize the result
                if(col % 3 < 2)
                    s_mat[block_mat_id][col + 1][col] = s_mat[block_mat_id][col][col + 1];
                else
                    s_mat[block_mat_id][col][col - 2] = s_mat[block_mat_id][col - 2][col];
                __syncthreads();

                // Write result back as float (mixed precision)
                for(int row = 0; row < MAT_DIM; row++)
                {
                    int node_row = row / 3;
                    int node_col = col / 3;
                    if(node_col >= node_row)
                    {
                        int si = sym_index(node_row, node_col);
                        cluster_inv[mat_id].M[si](row % 3, col % 3) =
                            static_cast<float>(s_mat[block_mat_id][row][col]);
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
void MASPreconditionerEngine::build_multi_level_R(const double3* R)
{
    using namespace muda;
    int N = m_total_map_nodes;
    if(N < 1) return;

    int block_size = DEFAULT_BLOCKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [R,
             multi_lr      = multi_level_R.data(),
             going_next    = going_next.data(),
             prefix_orig   = prefix_original.data(),
             fine_conn     = fine_connect_masks.data(),
             part_to_real  = part_to_real.cviewer().name("part_to_real"),
             level_num = m_level_num, N] __device__() mutable
            {
                int pdx = blockIdx.x * blockDim.x + threadIdx.x;
                if(pdx >= N) return;

                int idx = part_to_real(pdx);

                Eigen::Vector3f r;
                if(idx >= 0)
                {
                    r[0] = static_cast<float>(R[idx].x);
                    r[1] = static_cast<float>(R[idx].y);
                    r[2] = static_cast<float>(R[idx].z);
                }
                else
                {
                    r = Eigen::Vector3f::Zero();
                }

                int lane_id       = threadIdx.x % BANKSIZE;
                int local_warp_id = threadIdx.x / BANKSIZE;
                int global_warp   = pdx / BANKSIZE;

                multi_lr[pdx] = r;

                __shared__ float sum_residual[DEFAULT_BLOCKSIZE * 3];
                __shared__ int   prefix_sum_s[DEFAULT_WARPNUM];

                if(lane_id == 0)
                    prefix_sum_s[local_warp_id] = prefix_orig[global_warp];

                if(idx < 0) return;

                unsigned int connect_msk = fine_conn[idx];

                if(prefix_sum_s[local_warp_id] == 1)
                {
                    // Contiguous partition: use warp shuffle reduction
                    auto mask_val   = __activemask();
                    int  warp_id    = threadIdx.x & 0x1f;
                    bool is_boundary = (lane_id == 0) || (warp_id == 0);

                    unsigned int mark = __ballot_sync(mask_val, is_boundary);
                    mark              = __brev(mark);
                    int clz_len       = __clz(mark << (warp_id + 1));
                    unsigned int interval = min(clz_len, 31 - warp_id);

                    for(int s = 1; s < BANKSIZE; s <<= 1)
                    {
                        float tx = __shfl_down_sync(mask_val, r[0], s);
                        float ty = __shfl_down_sync(mask_val, r[1], s);
                        float tz = __shfl_down_sync(mask_val, r[2], s);
                        if(interval >= (unsigned int)s)
                        {
                            r[0] += tx;
                            r[1] += ty;
                            r[2] += tz;
                        }
                    }

                    if(is_boundary)
                    {
                        int cur = idx;
                        for(int l = 0; l < level_num - 1; l++)
                        {
                            cur = going_next[cur];
                            atomicAdd(&(multi_lr[cur][0]), r[0]);
                            atomicAdd(&(multi_lr[cur][1]), r[1]);
                            atomicAdd(&(multi_lr[cur][2]), r[2]);
                        }
                    }
                }
                else
                {
                    // Non-contiguous: accumulate via shared memory
                    int elected_lane = __ffs(connect_msk) - 1;

                    sum_residual[threadIdx.x]                         = 0;
                    sum_residual[threadIdx.x + DEFAULT_BLOCKSIZE]     = 0;
                    sum_residual[threadIdx.x + 2 * DEFAULT_BLOCKSIZE] = 0;

                    atomicAdd(sum_residual + local_warp_id * BANKSIZE + elected_lane, r[0]);
                    atomicAdd(sum_residual + local_warp_id * BANKSIZE + elected_lane + DEFAULT_BLOCKSIZE, r[1]);
                    atomicAdd(sum_residual + local_warp_id * BANKSIZE + elected_lane + 2 * DEFAULT_BLOCKSIZE, r[2]);

                    unsigned int elected_prefix = __popc(connect_msk & lanemask_lt(lane_id));
                    if(elected_prefix == 0)
                    {
                        int cur = idx;
                        for(int l = 0; l < level_num - 1; l++)
                        {
                            cur = going_next[cur];
                            atomicAdd(&(multi_lr[cur][0]), sum_residual[threadIdx.x]);
                            atomicAdd(&(multi_lr[cur][1]), sum_residual[threadIdx.x + DEFAULT_BLOCKSIZE]);
                            atomicAdd(&(multi_lr[cur][2]), sum_residual[threadIdx.x + DEFAULT_BLOCKSIZE * 2]);
                        }
                    }
                }
            });
}

// ---------------------------------------------------------------------------
// Local solve: Z = cluster_inverse * R at each level
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::schwarz_local_solve()
{
    using namespace muda;
    int N = m_total_num_clusters * BANKSIZE;  // one thread per (cluster, node-pair)
    if(N < 1) return;

    int block_size = BANKSIZE * BANKSIZE;
    int num_blocks = (N + block_size - 1) / block_size;

    Launch(num_blocks, block_size)
        .apply(
            [cluster_inv = cluster_inverses.data(),
             multi_lr    = multi_level_R.data(),
             multi_lz    = multi_level_Z.data(),
             N] __device__() mutable
            {
                int idx = blockIdx.x * blockDim.x + threadIdx.x;
                if(idx >= N) return;

                constexpr int cluster_stride = BANKSIZE * BANKSIZE;

                int cluster_id = idx / cluster_stride;
                int local_row  = (idx % cluster_stride) / BANKSIZE;
                int local_col  = (idx % cluster_stride) % BANKSIZE;

                int vert_row = cluster_id * BANKSIZE + local_row;
                int vert_col = cluster_id * BANKSIZE + local_col;

                // Load residual for this column node into shared memory
                __shared__ Eigen::Vector3f s_R[BANKSIZE];
                if(threadIdx.x < BANKSIZE)
                    s_R[threadIdx.x] = multi_lr[vert_col];
                __syncthreads();

                // Multiply: result = inverse_block * R_col
                Eigen::Vector3f result;
                if(vert_col >= vert_row)
                {
                    int si = sym_index(local_row, local_col);
                    result = cluster_inv[cluster_id].M[si] * s_R[local_col];
                }
                else
                {
                    int si = sym_index(local_col, local_row);
                    result = cluster_inv[cluster_id].M[si].transpose() * s_R[local_col];
                }

                // Warp-reduce the partial products across columns
                int  warp_id    = threadIdx.x & 0x1f;
                int  land_idx   = threadIdx.x % BANKSIZE;
                bool is_boundary = (land_idx == 0) || (warp_id == 0);

                unsigned int mark = __ballot_sync(0xffffffff, is_boundary);
                mark              = __brev(mark);
                int clz_len       = __clz(mark << (warp_id + 1));
                unsigned int interval = min(clz_len, 31 - warp_id);

                int max_shfl = min(32, BANKSIZE);
                for(int s = 1; s < max_shfl; s <<= 1)
                {
                    float tx = __shfl_down_sync(0xffffffff, result[0], s);
                    float ty = __shfl_down_sync(0xffffffff, result[1], s);
                    float tz = __shfl_down_sync(0xffffffff, result[2], s);
                    if(interval >= (unsigned int)s)
                    {
                        result[0] += tx;
                        result[1] += ty;
                        result[2] += tz;
                    }
                }

                if(is_boundary)
                {
                    atomicAdd(&(multi_lz[vert_row].x), result[0]);
                    atomicAdd(&(multi_lz[vert_row].y), result[1]);
                    atomicAdd(&(multi_lz[vert_row].z), result[2]);
                }
            });
}

// ---------------------------------------------------------------------------
// Prolongate: sum Z contributions from all levels for each fine node
// ---------------------------------------------------------------------------
void MASPreconditionerEngine::collect_final_Z(double3* Z)
{
    using namespace muda;
    int N = m_total_nodes;
    if(N < 1) return;

    int level_num = m_level_num;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [Z,
                multi_lz     = multi_level_Z.cviewer().name("multi_level_Z"),
                coarse_table = coarse_tables.cviewer().name("coarse_table"),
                real_to_part = real_to_part.cviewer().name("real_to_part"),
                level_num] __device__(int idx) mutable
               {
                   int rdx = real_to_part(idx);

                   // Skip unpartitioned vertices (handled by diagonal fallback)
                   if(rdx < 0) return;

                   // Start with the fine-level solution
                   float3 cz = multi_lz(rdx);

                   // Add contributions from all coarser levels
                   LevelTable table = coarse_table(idx);
                   for(int l = 1; l < level_num; l++)
                   {
                       int node = table.index[l - 1];
                       float3 val = multi_lz(node);
                       cz.x += val.x;
                       cz.y += val.y;
                       cz.z += val.z;
                   }

                   Z[idx].x = static_cast<double>(cz.x);
                   Z[idx].y = static_cast<double>(cz.y);
                   Z[idx].z = static_cast<double>(cz.z);
               });
}

// ============================================================================
// Apply: full preconditioning pipeline  z = M^{-1} r
// ============================================================================

void MASPreconditionerEngine::apply(muda::CDenseVectorView<Float> r,
                                    muda::DenseVectorView<Float>  z)
{
    if(m_total_nodes < 1)
        return;

    // Ensure multi-level buffers cover all clusters
    if(m_total_num_clusters > static_cast<int>(multi_level_R.size()))
    {
        multi_level_R.resize(m_total_num_clusters);
        multi_level_Z.resize(m_total_num_clusters);
    }

    // Zero coarse-level residuals (beyond fine nodes)
    if(m_total_num_clusters > m_total_map_nodes)
    {
        cudaMemset(multi_level_R.data() + m_total_map_nodes, 0,
                   (m_total_num_clusters - m_total_map_nodes) * sizeof(Eigen::Vector3f));
    }

    // Zero all solution levels
    cudaMemset(multi_level_Z.data(), 0,
               m_total_num_clusters * sizeof(float3));

    // 1. Restrict: accumulate residual down through levels
    build_multi_level_R(reinterpret_cast<const double3*>(r.data()));

    // 2. Local solve: Z = cluster_inverse * R at each level
    schwarz_local_solve();

    // 3. Prolongate: sum Z from all levels back to fine nodes
    collect_final_Z(reinterpret_cast<double3*>(z.data()));
}

}  // namespace uipc::backend::cuda
