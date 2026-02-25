#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <muda/ext/linear_system.h>

namespace uipc::backend::cuda
{
/**
 * @brief MAS (Multiplicative Additive Schwarz) preconditioner engine.
 *
 * Multi-level domain-decomposition preconditioner from StiffGIPC.
 * Operates on 3x3 block matrices (BCOO format) and produces z = M^{-1} r.
 *
 * Key parameters:
 * - BANKSIZE = 16: each cluster has at most 16 nodes (48 DOFs).
 * - Mixed precision: Hessian assembled in double, inverse stored in float.
 */
class MASPreconditionerEngine
{
  public:
    static constexpr int BANKSIZE          = 16;
    static constexpr int DEFAULT_BLOCKSIZE = 256;
    static constexpr int DEFAULT_WARPNUM   = 16;
    static constexpr int MAX_LEVELS        = 6;

    // Symmetric upper-triangle block count: BANKSIZE*(BANKSIZE+1)/2
    static constexpr int SYM_BLOCK_COUNT = BANKSIZE * (BANKSIZE + 1) / 2;

    // A single cluster's symmetric matrix in double (for assembly)
    struct alignas(16) ClusterMatrixSym
    {
        Eigen::Matrix3d M[SYM_BLOCK_COUNT];
    };

    // A single cluster's symmetric matrix in float (the inverted preconditioner)
    struct alignas(16) ClusterMatrixSymF
    {
        Eigen::Matrix3f M[SYM_BLOCK_COUNT];
    };

    // Level traversal table per node
    struct LevelTable
    {
        int index[MAX_LEVELS];
    };

    // Host-side level size cache (replaces CUDA int2 to avoid host-side CUDA types)
    struct Int2
    {
        int x = 0;
        int y = 0;
    };

    MASPreconditionerEngine()  = default;
    ~MASPreconditionerEngine() = default;

    // ---- Phase 1: Initialize neighbor structures (called once) ----

    void init_neighbor(int                              vert_num,
                       int                              total_neighbor_num,
                       int                              part_map_size,
                       const std::vector<unsigned int>& h_neighbor_list,
                       const std::vector<unsigned int>& h_neighbor_start,
                       const std::vector<unsigned int>& h_neighbor_num,
                       const std::vector<int>&          h_part_to_real,
                       const std::vector<int>&          h_real_to_part);

    // ---- Phase 1b: Allocate matrix-level buffers (called once) ----

    void init_matrix();

    // ---- Phase 2: Assemble preconditioner (per Newton iteration) ----

    void set_preconditioner(const Eigen::Matrix3d* d_triplet_values,
                            const int*             d_row_ids,
                            const int*             d_col_ids,
                            const uint32_t*        d_indices,
                            int                    dof_offset,
                            int                    triplet_num,
                            int                    cp_num);

    // ---- Phase 3: Apply preconditioning z = M^{-1} r (per PCG iteration) ----

    void apply(muda::CDenseVectorView<Float> r, muda::DenseVectorView<Float> z);

    bool is_initialized() const { return m_initialized; }

    // ===========================================================================
    // All methods below are public because NVCC on Windows requires
    // extended __device__ lambdas to reside in methods with public access.
    // ===========================================================================

    // Hierarchy building steps
    void compute_num_levels(int vert_num);
    int  reorder_realtime(int cp_num);
    void build_connect_mask_L0();
    void prepare_prefix_sum_L0();
    void build_level1();
    void build_connect_mask_Lx(int level);
    void next_level_cluster(int level);
    void prefix_sum_Lx(int level);
    void compute_next_level(int level);
    void aggregation_kernel();

    // Hessian assembly + inversion
    void scatter_hessian_to_clusters(const Eigen::Matrix3d* d_triplet_values,
                                     const int*             d_row_ids,
                                     const int*             d_col_ids,
                                     const uint32_t*        d_indices,
                                     int                    dof_offset,
                                     int                    triplet_num);
    void invert_cluster_matrices();

    // Preconditioning steps
    void build_multi_level_R(const double3* R);
    void schwarz_local_solve();
    void collect_final_Z(double3* Z);

  private:
    // ---- State ----
    bool m_initialized        = false;
    int  m_total_nodes        = 0;
    int  m_total_map_nodes    = 0;
    int  m_level_num          = 0;
    int  m_total_num_clusters = 0;
    Int2 m_h_level_size;

    // ---- GPU buffers: hierarchy ----
    muda::DeviceBuffer<Int2>         level_sizes;
    muda::DeviceBuffer<int>          coarse_space_tables;
    muda::DeviceBuffer<int>          prefix_original;
    muda::DeviceBuffer<int>          prefix_sum_original;
    muda::DeviceBuffer<int>          going_next;
    muda::DeviceBuffer<int>          dense_level;
    muda::DeviceBuffer<LevelTable>   coarse_tables;
    muda::DeviceBuffer<unsigned int> fine_connect_masks;
    muda::DeviceBuffer<unsigned int> next_connect_masks;
    muda::DeviceBuffer<unsigned int> next_prefixes;
    muda::DeviceBuffer<unsigned int> next_prefix_sums;

    // ---- GPU buffers: neighbor graph ----
    int m_neighbor_list_size = 0;
    muda::DeviceBuffer<unsigned int> neighbor_lists;
    muda::DeviceBuffer<unsigned int> neighbor_starts;
    muda::DeviceBuffer<unsigned int> neighbor_nums;
    muda::DeviceBuffer<unsigned int> neighbor_lists_init;
    muda::DeviceBuffer<unsigned int> neighbor_nums_init;

    // ---- GPU buffers: partition mappings ----
    muda::DeviceBuffer<int> part_to_real;   // partition-ordered index -> real vertex index
    muda::DeviceBuffer<int> real_to_part;   // real vertex index -> partition-ordered index

    // ---- GPU buffers: cluster matrices ----
    muda::DeviceBuffer<ClusterMatrixSym>  cluster_hessians;   // assembled Hessian blocks (double)
    muda::DeviceBuffer<ClusterMatrixSymF> cluster_inverses;   // inverted preconditioner (float)

    // ---- GPU buffers: multi-level residual / solution ----
    muda::DeviceBuffer<Eigen::Vector3f> multi_level_R;
    muda::DeviceBuffer<float3>          multi_level_Z;
};
}  // namespace uipc::backend::cuda
