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
 * This class encapsulates the GPU data structures and kernels for the
 * multi-level Schwarz preconditioner from StiffGIPC. It operates on
 * 3x3 block matrices (BCOO format) and produces a preconditioned solution
 * z = M^{-1} r in each PCG iteration.
 *
 * Key parameters:
 * - BANKSIZE = 16: Each cluster has at most 16 nodes (48 DOFs).
 * - Mixed precision: Hessian assembled in double, inverse stored in float.
 */
class MASPreconditionerEngine
{
  public:
    static constexpr int BANKSIZE         = 16;
    static constexpr int DEFAULT_BLOCKSIZE = 256;
    static constexpr int DEFAULT_WARPNUM  = 16;
    static constexpr int MAX_LEVELS       = 6;

    // Symmetric upper-triangle block: BANKSIZE*(BANKSIZE+1)/2 blocks of Matrix3d
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

    // Host-side level size cache
    struct Int2
    {
        int x = 0;
        int y = 0;
    };

    MASPreconditionerEngine()  = default;
    ~MASPreconditionerEngine() = default;

    // Phase 1: Initialize neighbor structures (called once)
    void init_neighbor(int   vertNum,
                       int   totalNeighborNum,
                       int   partMapSize,
                       const std::vector<unsigned int>& h_neighborList,
                       const std::vector<unsigned int>& h_neighborStart,
                       const std::vector<unsigned int>& h_neighborNum,
                       const std::vector<int>&          h_partId_map_real,
                       const std::vector<int>&          h_real_map_partId);

    // Phase 1b: Allocate matrix-level buffers (called once after init_neighbor)
    void init_matrix();

    // Phase 2: Assemble preconditioner from BCOO Hessian (per Newton iteration)
    void set_preconditioner(const Eigen::Matrix3d* d_triplet_values,
                            const int*             d_row_ids,
                            const int*             d_col_ids,
                            const uint32_t*        d_indices,
                            int                    dof_offset,
                            int                    triplet_num,
                            int                    cpNum);

    // Phase 3: Apply preconditioning z = M^{-1} r (per PCG iteration)
    void apply(muda::CDenseVectorView<Float> r, muda::DenseVectorView<Float> z);

    bool is_initialized() const { return m_initialized; }

    // NOTE: These must be public because NVCC requires methods containing
    // extended __device__ lambdas to have public access.
    void prepare_hessian_bcoo(const Eigen::Matrix3d* d_triplet_values,
                              const int*             d_row_ids,
                              const int*             d_col_ids,
                              const uint32_t*        d_indices,
                              int                    dof_offset,
                              int                    triplet_num);

  private:
    // Hierarchy building
    void compute_num_levels(int vertNum);
    int  reorder_realtime(int cpNum);
    void build_connect_mask_L0();
    void prepare_prefix_sum_L0();
    void build_level1();
    void build_connect_mask_Lx(int level);
    void next_level_cluster(int level);
    void prefix_sum_Lx(int level);
    void compute_next_level(int level);
    void aggregation_kernel();

    // Preconditioning steps
    void build_multi_level_R(const double3* R);
    void schwarz_local_solve();
    void collect_final_Z(double3* Z);

    // State
    bool m_initialized = false;
    int  m_totalNodes          = 0;
    int  m_totalMapNodes       = 0;
    int  m_levelnum            = 0;
    int  m_totalNumberClusters = 0;
    Int2 m_h_clevelSize;

    // GPU buffers — hierarchy
    muda::DeviceBuffer<Int2>         m_d_levelSize;
    muda::DeviceBuffer<int>          m_d_coarseSpaceTables;
    muda::DeviceBuffer<int>          m_d_prefixOriginal;
    muda::DeviceBuffer<int>          m_d_prefixSumOriginal;
    muda::DeviceBuffer<int>          m_d_goingNext;
    muda::DeviceBuffer<int>          m_d_denseLevel;
    muda::DeviceBuffer<LevelTable>   m_d_coarseTable;
    muda::DeviceBuffer<unsigned int> m_d_fineConnectMask;
    muda::DeviceBuffer<unsigned int> m_d_nextConnectMask;
    muda::DeviceBuffer<unsigned int> m_d_nextPrefix;
    muda::DeviceBuffer<unsigned int> m_d_nextPrefixSum;

    // GPU buffers — neighbor graph
    int m_neighborListSize = 0;
    muda::DeviceBuffer<unsigned int> m_d_neighborList;
    muda::DeviceBuffer<unsigned int> m_d_neighborStart;
    muda::DeviceBuffer<unsigned int> m_d_neighborNum;
    muda::DeviceBuffer<unsigned int> m_d_neighborListInit;
    muda::DeviceBuffer<unsigned int> m_d_neighborNumInit;

    // GPU buffers — partition mappings
    muda::DeviceBuffer<int> m_d_partId_map_real;
    muda::DeviceBuffer<int> m_d_real_map_partId;

    // GPU buffers — cluster matrices
    muda::DeviceBuffer<ClusterMatrixSym>  m_d_inverseMatMas;   // assembled Hessian blocks (double)
    muda::DeviceBuffer<ClusterMatrixSymF> m_d_precondMatMas;   // inverted preconditioner (float)

    // GPU buffers — multi-level residual / solution
    muda::DeviceBuffer<Eigen::Vector3f> m_d_multiLevelR;
    muda::DeviceBuffer<float3>          m_d_multiLevelZ;
};
}  // namespace uipc::backend::cuda
