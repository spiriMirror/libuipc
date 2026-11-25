/*
 * NVIDIA contributions: interfaces and state supporting MAS direct-store and
 * vector-pass specializations, the prepared cluster clear, the symmetric-sweep
 * cluster inversion, the empty-bank skip, the fused step with the diagonal
 * fallback and the preconditioner's launch streams.
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

#pragma once
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>
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
 * - Mixed precision (matches StiffGIPC):
 *     - Hessian assembly and Gauss-Jordan inversion run in double.
 *     - The inverted per-cluster preconditioner, multi-level residual and
 *       solution buffers are stored in float for memory / atomic bandwidth.
 *     - Only the final apply output is cast back to double.
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
        UIPC_GENERIC                ClusterMatrixSymT()
        {
            for(auto& m : M)
                m.setZero();
        }
    };

    using ClusterMatrixSym = ClusterMatrixSymT<double>;  // Hessian assembly
    using ClusterMatrixSymF = ClusterMatrixSymT<float>;  // Inverted preconditioner

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

    void init_neighbor(int                      vert_num,
                       int                      total_neighbor_num,
                       int                      part_map_size,
                       span<const unsigned int> h_neighbor_list,
                       span<const unsigned int> h_neighbor_start,
                       span<const unsigned int> h_neighbor_num,
                       span<const int>          h_part_to_real,
                       span<const int>          h_real_to_part);

    // ---- Phase 1b: Allocate matrix-level buffers (called once) ----

    void init_matrix();

    // ---- Phase 2: Assemble preconditioner (per Newton iteration) ----

    // Optional, before set_preconditioner(): clear the cluster Hessians on a
    // side stream, ordered after the work enqueued on the default stream so
    // far, so the clear runs beside whatever the default stream does until
    // set_preconditioner(), which then waits for it instead of clearing.
    void prepare_preconditioner();

    void set_preconditioner(cuda_tool::CBufferView<Eigen::Matrix3d> triplet_values,
                            cuda_tool::CBufferView<int> row_ids,
                            cuda_tool::CBufferView<int> col_ids,
                            int                         dof_offset);

    // ---- Phase 3: Apply preconditioning z = M^{-1} r (per PCG iteration) ----

    // Unpartitioned vertices (no MAS cluster) get z = D^{-1} r from the
    // diagonal fallback `unpart_diag_inv` (one inverse block per FEM vertex,
    // as FEMMASPreconditioner assembles it) in the prolongation pass; with an
    // empty view their z is left untouched.
    void apply(cuda_tool::CDenseVectorView<Float> r,
               cuda_tool::DenseVectorView<Float>  z,
               cuda_tool::CVarView<IndexT>        converged,
               cudaStream_t                       stream          = nullptr,
               cuda_tool::CBufferView<Matrix3x3>  unpart_diag_inv = {});

    /**
     * @brief Execute the PCG x/r update, MAS apply, and r^T z accumulation.
     *
     * This is the graph-friendly MAS path. The caller must provide a zeroed
     * rz_new accumulation slot. Unpartitioned vertices (no MAS cluster) need
     * the per-vertex diagonal fallback `unpart_diag_inv` (one inverse block
     * per FEM vertex, as FEMMASPreconditioner assembles it); an empty view
     * means every vertex is partitioned.
     */
    void fused_pcg_update_apply_dot(cuda_tool::DenseVectorView<Float>  x,
                                    cuda_tool::CDenseVectorView<Float> p,
                                    cuda_tool::DenseVectorView<Float>  r,
                                    cuda_tool::CDenseVectorView<Float> Ap,
                                    cuda_tool::DenseVectorView<Float>  z,
                                    cuda_tool::CVarView<Float>         rz,
                                    cuda_tool::CVarView<Float>         pAp,
                                    cuda_tool::VarView<Float>          rz_new,
                                    cuda_tool::CVarView<IndexT> converged,
                                    cudaStream_t stream = nullptr,
                                    cuda_tool::CBufferView<Matrix3x3> unpart_diag_inv = {});

    bool is_initialized() const { return m_initialized; }

    /**
     * @brief Override the number of active levels used during apply().
     *
     * By default all m_level_num levels are used (full hierarchy, injection prolongation).
     * Set to 1 to use only the fine level (block Jacobi with BANKSIZE-node clusters).
     *
     * @param n  1 = fine-only (block Jacobi), 0 = use full hierarchy (default).
     */
    void set_active_level_num(int n)
    {
        if(n <= 0 || n > m_level_num)
            m_active_level_num = m_level_num;  // clamp: 0 or out-of-range -> full hierarchy
        else
            m_active_level_num = n;
    }

    /**
     * @brief No-op kept for API compatibility; coarse damping is unused since
     *        collect_final_Z now uses pure injection prolongation (StiffGIPC default).
     */
    void set_coarse_damping(Float /*omega*/) {}

    /** Dump cluster matrices in Matrix Market (.mtx) format and metadata as JSON for debug.
     *
     * `output_dir` is interpreted as a filesystem path; the caller may pass any
     * `std::filesystem::path::string()`-like result. Files written:
     *   - `mas_cluster_hess.f<frame>.n<newton>.mtx`
     *   - `mas_cluster_inv.f<frame>.n<newton>.mtx`
     *   - `mas_cluster_meta.f<frame>.n<newton>.json`
     */
    void dump_cluster_matrices_debug(std::string_view output_dir, SizeT frame, SizeT newton_iter);

    // ===========================================================================
    // All methods below are public because NVCC on Windows requires
    // extended __device__ lambdas to reside in methods with public access.
    // ===========================================================================

    // Hierarchy building steps
    void compute_num_levels(int vert_num);
    int  reorder_realtime();
    void build_connect_mask_L0();
    void prepare_prefix_sum_L0();
    void build_level1();
    void build_connect_mask_Lx(int level);
    void next_level_cluster(int level);
    void prefix_sum_Lx(int level);
    void compute_next_level(int level);
    void aggregation_kernel();

    // Hessian assembly + inversion
    void scatter_hessian_to_clusters(cuda_tool::CBufferView<Eigen::Matrix3d> triplet_values,
                                     cuda_tool::CBufferView<int> row_ids,
                                     cuda_tool::CBufferView<int> col_ids,
                                     int                         dof_offset);
    void invert_cluster_matrices();
    // cluster_inv(k) = inverse of cluster_hess(k) (its upper triangle) for
    // the first cluster_count clusters except the skip_count clusters from
    // skip_begin on, whose inverses are left untouched (one 96-thread block
    // per inverted 48x48 cluster)
    static void invert_cluster_matrices(cuda_tool::CBufferView<ClusterMatrixSym> cluster_hess,
                                        cuda_tool::BufferView<ClusterMatrixSymF> cluster_inv,
                                        int          cluster_count,
                                        cudaStream_t stream,
                                        int          skip_begin = 0,
                                        int          skip_count = 0);

    // Preconditioning steps
    void build_multi_level_R(cuda_tool::CDenseVectorView<Float> R,
                             cuda_tool::CVarView<IndexT>        converged,
                             cudaStream_t stream = nullptr);
    void schwarz_local_solve(cuda_tool::CVarView<IndexT> converged,
                             cudaStream_t                stream = nullptr);
    // With a non-empty `unpart_diag_inv` (see apply()) the pass also writes
    // Z = D^{-1} R for the unpartitioned vertices.
    void collect_final_Z(cuda_tool::DenseVectorView<Float>  Z,
                         cuda_tool::CVarView<IndexT>        converged,
                         cudaStream_t                       stream = nullptr,
                         cuda_tool::CDenseVectorView<Float> R      = {},
                         cuda_tool::CBufferView<Matrix3x3> unpart_diag_inv = {});

  private:
    friend class MASPreconditionerEngineTestAccess;

    // Clusters of level 0, and of the empty banks after it (m_level1_begin)
    int level0_cluster_count() const;
    int empty_cluster_count() const;
    // Cluster blocks set_preconditioner() assembles (0: nothing to assemble)
    int assembled_cluster_count() const;
    // Zero the cluster Hessians that the scatter accumulates into
    void clear_cluster_hessians(int num_cluster_blocks, cudaStream_t stream);

    // ---- State ----
    bool m_initialized     = false;
    int  m_total_nodes     = 0;
    int  m_total_map_nodes = 0;
    int  m_level_num       = 0;
    int m_active_level_num = 0;  // levels used during apply(); 0 = use m_level_num
    int m_total_num_clusters = 0;
    // First node of level 1. Level 0 occupies the nodes [0, m_total_map_nodes)
    // (bank aligned); the banks between there and m_level1_begin, which
    // exist when unpartitioned vertices make m_total_nodes larger, belong to
    // no level: no Hessian entry, residual or prolongation ever reaches them.
    int  m_level1_begin = 0;
    Int2 m_h_level_size;

    // ---- GPU buffers: hierarchy ----
    cuda_tool::DeviceBuffer<Int2>         level_sizes;
    cuda_tool::DeviceBuffer<int>          coarse_space_tables;
    cuda_tool::DeviceBuffer<int>          prefix_original;
    cuda_tool::DeviceBuffer<int>          prefix_sum_original;
    cuda_tool::DeviceBuffer<int>          going_next;
    cuda_tool::DeviceBuffer<int>          dense_level;
    cuda_tool::DeviceBuffer<LevelTable>   coarse_tables;
    cuda_tool::DeviceBuffer<unsigned int> fine_connect_masks;
    cuda_tool::DeviceBuffer<unsigned int> next_connect_masks;
    cuda_tool::DeviceBuffer<unsigned int> next_prefixes;
    cuda_tool::DeviceBuffer<unsigned int> next_prefix_sums;

    // ---- GPU buffers: neighbor graph ----
    int                                   m_neighbor_list_size = 0;
    cuda_tool::DeviceBuffer<unsigned int> neighbor_lists;
    cuda_tool::DeviceBuffer<unsigned int> neighbor_starts;
    cuda_tool::DeviceBuffer<unsigned int> neighbor_nums;
    cuda_tool::DeviceBuffer<unsigned int> neighbor_lists_init;
    cuda_tool::DeviceBuffer<unsigned int> neighbor_nums_init;

    // ---- GPU buffers: partition mappings ----
    cuda_tool::DeviceBuffer<int> part_to_real;  // partition-ordered index -> real vertex index
    cuda_tool::DeviceBuffer<int> real_to_part;  // real vertex index -> partition-ordered index

    // ---- GPU buffers: cluster matrices ----
    cuda_tool::DeviceBuffer<ClusterMatrixSym> cluster_hessians;  // assembled Hessian blocks (double)
    cuda_tool::DeviceBuffer<ClusterMatrixSymF> cluster_inverses;  // inverted preconditioner (float, matches GIPC)
    // prepare_preconditioner() cleared cluster_hessians on this stream;
    // the next set_preconditioner() joins that stream instead of clearing
    cuda_tool::SideStream m_hessian_clear_stream;
    bool                  m_hessians_cleared = false;

    // ---- GPU buffers: multi-level residual / solution (float, matches GIPC) ----
    cuda_tool::DeviceBuffer<Eigen::Vector3f> multi_level_R;
    cuda_tool::DeviceBuffer<float3>          multi_level_Z;
    // The coarse levels of multi_level_R (from m_level1_begin on) are
    // zero in stream order: the restriction accumulates into them and the
    // Schwarz local solve clears them again after reading. False until the
    // first clear after an allocation; apply() always clears.
    bool m_coarse_R_zero = false;
};
}  // namespace uipc::backend::cuda
