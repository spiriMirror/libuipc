#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <muda/ext/linear_system.h>
#include <uipc/common/span.h>
#include <filesystem>
#include <string_view>

namespace uipc::backend::cuda
{
/**
 * @brief MAS (MultiLevel Additive Schwarz) preconditioner engine.
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

    template <typename Scalar>
    struct alignas(16) ClusterMatrixSymT
    {
        Eigen::Matrix<Scalar, 3, 3> M[SYM_BLOCK_COUNT];
        MUDA_GENERIC                ClusterMatrixSymT()
        {
            for(auto& m : M)
                m.setZero();
        }
    };

    using ClusterMatrixSym  = ClusterMatrixSymT<double>;  // Hessian assembly
    using ClusterMatrixSymF = ClusterMatrixSymT<float>;   // Inverted preconditioner

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

    void init_neighbor(int                     vert_num,
                       int                     total_neighbor_num,
                       int                     part_map_size,
                       span<const unsigned int> h_neighbor_list,
                       span<const unsigned int> h_neighbor_start,
                       span<const unsigned int> h_neighbor_num,
                       span<const int>          h_part_to_real,
                       span<const int>          h_real_to_part);

    // ---- Phase 1b: Allocate matrix-level buffers (called once) ----

    void init_matrix();

    // ---- Phase 2: Assemble preconditioner (per Newton iteration) ----

    void set_preconditioner(muda::CBufferView<Eigen::Matrix3d> triplet_values,
                            muda::CBufferView<int>             row_ids,
                            muda::CBufferView<int>             col_ids,
                            muda::CBufferView<uint32_t>        indices,
                            int                                dof_offset,
                            int                                cp_num);

    // ---- Phase 3: Apply preconditioning z = M^{-1} r (per PCG iteration) ----

    void apply(muda::CDenseVectorView<Float> r,
               muda::DenseVectorView<Float>  z,
               muda::CVarView<IndexT>        converged);

    bool is_initialized() const { return m_initialized; }

    /** Dump cluster matrices in Matrix Market (.mtx) format and metadata as plain text for debug. */
    void dump_cluster_matrices_debug(const std::filesystem::path& output_dir,
                                     std::string_view             label,
                                     SizeT                        frame,
                                     SizeT                        newton_iter);

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
    void scatter_hessian_to_clusters(muda::CBufferView<Eigen::Matrix3d> triplet_values,
                                     muda::CBufferView<int>             row_ids,
                                     muda::CBufferView<int>             col_ids,
                                     muda::CBufferView<uint32_t>        indices,
                                     int                                dof_offset);
    void invert_cluster_matrices();

    // Preconditioning steps
    void build_multi_level_R(muda::CDenseVectorView<Float> R, muda::CVarView<IndexT> converged);
    void schwarz_local_solve(muda::CVarView<IndexT> converged);
    void collect_final_Z(muda::DenseVectorView<Float> Z, muda::CVarView<IndexT> converged);

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
    int                              m_neighbor_list_size = 0;
    muda::DeviceBuffer<unsigned int> neighbor_lists;
    muda::DeviceBuffer<unsigned int> neighbor_starts;
    muda::DeviceBuffer<unsigned int> neighbor_nums;
    muda::DeviceBuffer<unsigned int> neighbor_lists_init;
    muda::DeviceBuffer<unsigned int> neighbor_nums_init;

    // ---- GPU buffers: partition mappings ----
    muda::DeviceBuffer<int> part_to_real;  // partition-ordered index -> real vertex index
    muda::DeviceBuffer<int> real_to_part;  // real vertex index -> partition-ordered index

    // ---- GPU buffers: cluster matrices ----
    muda::DeviceBuffer<ClusterMatrixSym> cluster_hessians;  // assembled Hessian blocks (double)
    muda::DeviceBuffer<ClusterMatrixSymF> cluster_inverses;  // inverted preconditioner (float)

    // ---- GPU buffers: multi-level residual / solution ----
    muda::DeviceBuffer<Eigen::Vector3f> multi_level_R;
    muda::DeviceBuffer<float3>          multi_level_Z;
};
}  // namespace uipc::backend::cuda
