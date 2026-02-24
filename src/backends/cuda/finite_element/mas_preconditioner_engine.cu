#include <finite_element/mas_preconditioner_engine.h>
#include <muda/launch/launch.h>
#include <muda/launch/parallel_for.h>
#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include <cooperative_groups.h>
#include <cooperative_groups/reduce.h>

namespace uipc::backend::cuda
{

// ============================================================================
// Constants
// ============================================================================
static constexpr int BANKSIZE          = MASPreconditionerEngine::BANKSIZE;
static constexpr int DEFAULT_BLOCKSIZE = MASPreconditionerEngine::DEFAULT_BLOCKSIZE;
static constexpr int DEFAULT_WARPNUM   = MASPreconditionerEngine::DEFAULT_WARPNUM;
static constexpr int SYM_BLOCK_COUNT   = MASPreconditionerEngine::SYM_BLOCK_COUNT;

using ClusterMatrixSym  = MASPreconditionerEngine::ClusterMatrixSym;
using ClusterMatrixSymF = MASPreconditionerEngine::ClusterMatrixSymF;
using LevelTable        = MASPreconditionerEngine::LevelTable;
using Int2              = MASPreconditionerEngine::Int2;

// ============================================================================
// Device helper: lane mask for bits less than laneIdx
// ============================================================================
__device__ unsigned int lanemask_lt(int laneIdx)
{
    return (1U << laneIdx) - 1;
}

// ============================================================================
// Kernel: Build connect mask at level 0 (with partition reorder)
// ============================================================================
__global__ void k_buildCML0(const unsigned int* __restrict__ neighborStart,
                            unsigned int* __restrict__ neighborNum,
                            unsigned int* __restrict__ neighborList,
                            unsigned int* __restrict__ fineConnectedMsk,
                            const int* __restrict__ partId_map_real,
                            const int* __restrict__ real_map_partId,
                            int number)
{
    int tdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(tdx >= number) return;

    int warpId = tdx / BANKSIZE;
    int laneId = tdx % BANKSIZE;
    int idx    = partId_map_real[tdx];

    if(idx >= 0)
    {
        int          numNeighbor = neighborNum[idx];
        unsigned int connectMsk  = (1U << laneId);
        int          nk          = 0;
        int          startId     = neighborStart[idx];

        for(int i = 0; i < numNeighbor; i++)
        {
            int vIdConnected     = neighborList[startId + i];
            int warpIdxConnected = real_map_partId[vIdConnected] / BANKSIZE;
            if(warpId == warpIdxConnected)
            {
                unsigned int laneIdxConnected = real_map_partId[vIdConnected] % BANKSIZE;
                connectMsk |= (1U << laneIdxConnected);
            }
            else
            {
                neighborList[startId + nk] = vIdConnected;
                nk++;
            }
        }
        neighborNum[idx]      = nk;
        fineConnectedMsk[idx] = connectMsk;
    }
}

// ============================================================================
// Kernel: Prepare prefix sum at level 0
// ============================================================================
__global__ void k_preparePrefixSumL0(int* __restrict__ prefixOriginal,
                                     unsigned int* __restrict__ fineConnectedMsk,
                                     const int* __restrict__ partId_map_real,
                                     int number)
{
    int tdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(tdx >= number) return;

    int warpId      = tdx / BANKSIZE;
    int localWarpId = threadIdx.x / BANKSIZE;
    int laneId      = tdx % BANKSIZE;
    int idx         = partId_map_real[tdx];

    __shared__ unsigned int cacheMask[DEFAULT_BLOCKSIZE];
    __shared__ int          prefixSum[DEFAULT_WARPNUM];

    if(idx >= 0)
    {
        unsigned int connectMsk = fineConnectedMsk[idx];
        if(laneId == 0) prefixSum[localWarpId] = 0;
        cacheMask[threadIdx.x] = connectMsk;
        unsigned int visited   = (1U << laneId);

        while(connectMsk != ~0U)
        {
            unsigned int todo = visited ^ connectMsk;
            if(!todo) break;
            unsigned int nextVist = __ffs(todo) - 1;
            visited |= (1U << nextVist);
            connectMsk |= cacheMask[nextVist + localWarpId * BANKSIZE];
        }

        fineConnectedMsk[idx] = connectMsk;

        unsigned int electedPrefix = __popc(connectMsk & lanemask_lt(laneId));
        if(electedPrefix == 0) atomicAdd(prefixSum + localWarpId, 1);
        if(laneId == 0) prefixOriginal[warpId] = prefixSum[localWarpId];
    }
}

// ============================================================================
// Kernel: Build level 1
// ============================================================================
__global__ void k_buildLevel1(Int2* __restrict__ levelSize,
                              int* __restrict__ coarseSpaceTable,
                              int* __restrict__ goingNext,
                              const unsigned int* __restrict__ fineConnectedMsk,
                              const int* __restrict__ prefixSumOriginal,
                              const int* __restrict__ prefixOriginal,
                              const int* __restrict__ partId_map_real,
                              int number)
{
    int tdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(tdx >= number) return;

    int warpId      = tdx / BANKSIZE;
    int localWarpId = threadIdx.x / BANKSIZE;
    int laneId      = tdx % BANKSIZE;

    __shared__ unsigned int electedMask[BANKSIZE];
    __shared__ unsigned int lanePrefix[BANKSIZE * BANKSIZE];

    if(laneId == 0) electedMask[localWarpId] = 0;

    if(tdx == number - 1)
    {
        levelSize[1].x = prefixSumOriginal[warpId] + prefixOriginal[warpId];
        levelSize[1].y = (number + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
    }

    int idx = partId_map_real[tdx];
    if(idx >= 0)
    {
        unsigned int connMsk       = fineConnectedMsk[idx];
        unsigned int electedPrefix = __popc(connMsk & lanemask_lt(laneId));
        if(electedPrefix == 0) atomicOr(electedMask + localWarpId, (1U << laneId));

        lanePrefix[threadIdx.x] = __popc(electedMask[localWarpId] & lanemask_lt(laneId));
        lanePrefix[threadIdx.x] += prefixSumOriginal[warpId];

        unsigned int elected_lane  = __ffs(connMsk) - 1;
        unsigned int theLanePrefix = lanePrefix[elected_lane + BANKSIZE * localWarpId];

        coarseSpaceTable[idx] = theLanePrefix;
        goingNext[idx] = theLanePrefix + (number + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
    }
}

// ============================================================================
// Kernel: Build connect mask at level x
// ============================================================================
__global__ void k_buildConnectMaskLx(const unsigned int* __restrict__ neighborStart,
                                     unsigned int* __restrict__ neighborNum,
                                     unsigned int* __restrict__ neighborList,
                                     const int* __restrict__ coarseSpaceTable,
                                     unsigned int* __restrict__ nextConnectedMsk,
                                     const unsigned int* __restrict__ fineConnectedMsk,
                                     int level,
                                     const int* __restrict__ partId_map_real,
                                     int vertNum,
                                     int number)
{
    int tdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(tdx >= number) return;

    int localWarpId = threadIdx.x / BANKSIZE;
    int laneId      = tdx % BANKSIZE;
    __shared__ int cacheMsk[DEFAULT_BLOCKSIZE];

    int idx = partId_map_real[tdx];
    if(idx >= 0)
    {
        unsigned int prefixMsk = fineConnectedMsk[idx];
        unsigned int connMsk   = 0;
        unsigned int coarseIdx = coarseSpaceTable[(level - 1) * vertNum + idx];
        int          kn        = neighborNum[idx];
        int          nk        = 0;
        int          startId   = neighborStart[idx];

        for(int i = 0; i < kn; i++)
        {
            unsigned int connect       = neighborList[startId + i];
            unsigned int coarseConnect = coarseSpaceTable[(level - 1) * vertNum + connect];
            if(coarseIdx / BANKSIZE == coarseConnect / BANKSIZE)
            {
                unsigned int off = coarseConnect % BANKSIZE;
                connMsk |= (1U << off);
            }
            else
            {
                neighborList[startId + nk] = connect;
                nk++;
            }
        }
        neighborNum[idx] = nk;
        cacheMsk[threadIdx.x] = 0;

        if(__popc(prefixMsk) == BANKSIZE)
        {
            atomicOr(cacheMsk + localWarpId * BANKSIZE, (int)connMsk);
            connMsk = (unsigned int)cacheMsk[localWarpId * BANKSIZE];
        }
        else
        {
            unsigned int electedLane = __ffs(prefixMsk) - 1;
            if(connMsk)
                atomicOr(cacheMsk + localWarpId * BANKSIZE + electedLane, (int)connMsk);
            connMsk = (unsigned int)cacheMsk[localWarpId * BANKSIZE + electedLane];
        }

        unsigned int electedPrefix = __popc(prefixMsk & lanemask_lt(laneId));
        if(connMsk && electedPrefix == 0)
            atomicOr(nextConnectedMsk + coarseIdx, connMsk);
    }
}

// ============================================================================
// Kernel: Next level cluster
// ============================================================================
__global__ void k_nextLevelCluster(unsigned int* __restrict__ nextConnectedMsk,
                                   unsigned int* __restrict__ nextPrefix,
                                   int number)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= number) return;

    int localWarpId = threadIdx.x / BANKSIZE;
    int laneId      = idx % BANKSIZE;

    __shared__ int          prefixSum[DEFAULT_WARPNUM];
    __shared__ unsigned int cachedMsk[DEFAULT_BLOCKSIZE];

    if(laneId == 0) prefixSum[localWarpId] = 0;

    unsigned int connMsk = (1U << laneId);
    connMsk |= nextConnectedMsk[idx];
    cachedMsk[threadIdx.x] = connMsk;
    unsigned int visited = (1U << laneId);

    while(true)
    {
        unsigned int todo = visited ^ connMsk;
        if(!todo) break;
        unsigned int nextVisit = __ffs(todo) - 1;
        visited |= (1U << nextVisit);
        connMsk |= cachedMsk[nextVisit + localWarpId * BANKSIZE];
    }

    nextConnectedMsk[idx] = connMsk;
    unsigned int electedPrefix = __popc(connMsk & lanemask_lt(laneId));
    if(electedPrefix == 0) atomicAdd(prefixSum + localWarpId, 1);
    if(laneId == 0) nextPrefix[idx / BANKSIZE] = prefixSum[localWarpId];
}

// ============================================================================
// Kernel: Prefix sum at level x
// ============================================================================
__global__ void k_prefixSumLx(Int2* __restrict__ levelSize,
                              unsigned int* __restrict__ nextPrefix,
                              unsigned int* __restrict__ nextPrefixSum,
                              unsigned int* __restrict__ nextConnectMsk,
                              int* __restrict__ goingNext,
                              int level, int levelBegin, int number)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= number) return;

    int warpId      = idx / BANKSIZE;
    int localWarpId = threadIdx.x / BANKSIZE;
    int laneId      = idx % BANKSIZE;

    __shared__ unsigned int electedMask[BANKSIZE];
    __shared__ unsigned int lanePrefix[BANKSIZE * BANKSIZE];

    if(laneId == 0) electedMask[localWarpId] = 0;

    if(idx == number - 1)
    {
        levelSize[level + 1].x = nextPrefixSum[warpId] + nextPrefix[warpId];
        levelSize[level + 1].y = levelBegin + (number + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
    }

    unsigned int connMsk = nextConnectMsk[idx];
    unsigned int electedPrefix = __popc(connMsk & lanemask_lt(laneId));
    if(electedPrefix == 0) atomicOr(electedMask + localWarpId, (1U << laneId));

    lanePrefix[threadIdx.x] = __popc(electedMask[localWarpId] & lanemask_lt(laneId));
    lanePrefix[threadIdx.x] += nextPrefixSum[warpId];

    unsigned int elected_lane  = __ffs(connMsk) - 1;
    unsigned int theLanePrefix = lanePrefix[elected_lane + BANKSIZE * localWarpId];

    nextConnectMsk[idx] = theLanePrefix;
    goingNext[idx + levelBegin] =
        theLanePrefix + levelBegin + (number + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
}

// ============================================================================
// Kernel: Compute next level
// ============================================================================
__global__ void k_computeNextLevel(int* __restrict__ coarseSpaceTable,
                                   const unsigned int* __restrict__ nextConnectMsk,
                                   int level, int number)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= number) return;
    int next = coarseSpaceTable[(level - 1) * number + idx];
    coarseSpaceTable[level * number + idx] = nextConnectMsk[next];
}

// ============================================================================
// Kernel: Aggregation
// ============================================================================
__global__ void k_aggregationKernel(int* __restrict__ denseLevel,
                                    LevelTable* __restrict__ coarseTable,
                                    const int* __restrict__ goingNext,
                                    int levelNum, int number)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= number) return;

    int        currentId = idx;
    LevelTable ctable;
    for(int l = 0; l < levelNum - 1; l++)
    {
        int next      = goingNext[currentId];
        currentId     = next;
        ctable.index[l] = next;
    }
    coarseTable[idx] = ctable;
}

// ============================================================================
// Kernel: Invert cluster matrix (Gauss-Jordan on 48x48 or smaller)
// ============================================================================
__global__ void k_invertClusterMatrix(ClusterMatrixSymF* __restrict__ preMatrix,
                                      const ClusterMatrixSym* __restrict__ invMatrix,
                                      int numbers)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= numbers) return;

    int matId       = idx / (BANKSIZE * 3);
    int i           = idx % (BANKSIZE * 3);
    int block_matId = threadIdx.x / (BANKSIZE * 3);

    __shared__ double sPMas[32 / BANKSIZE][BANKSIZE * 3][BANKSIZE * 3];
    __shared__ double colm[32 / BANKSIZE][BANKSIZE * 3];

    for(int j = 0; j < (BANKSIZE * 3); j++)
    {
        int rowId = j / 3;
        int colId = i / 3;
        int index = 0;
        if(colId >= rowId)
        {
            index = BANKSIZE * rowId - rowId * (rowId + 1) / 2 + colId;
            sPMas[block_matId][j][i] = invMatrix[matId].M[index](j % 3, i % 3);
        }
        else
        {
            index = BANKSIZE * colId - colId * (colId + 1) / 2 + rowId;
            sPMas[block_matId][j][i] = invMatrix[matId].M[index](i % 3, j % 3);
        }
        if(i == j)
        {
            if(sPMas[block_matId][j][i] == 0.0)
                sPMas[block_matId][j][i] = 1.0;
        }
    }

    int    j = 0;
    double rt;

    while(j < (BANKSIZE * 3))
    {
        __syncthreads();
        rt = sPMas[block_matId][j][j];
        colm[block_matId][i] = sPMas[block_matId][i][j];
        __syncthreads();

        if(i == j)
            sPMas[block_matId][i][j] = 1.0;
        else
            sPMas[block_matId][i][j] = 0.0;

        __syncthreads();
        sPMas[block_matId][j][i] /= rt;
        __syncthreads();

        for(int k = 0; k < (BANKSIZE * 3); k++)
        {
            if(k != j)
            {
                double rate = -colm[block_matId][k];
                __syncthreads();
                sPMas[block_matId][k][i] += rate * sPMas[block_matId][j][i];
            }
        }
        j++;
    }

    __syncthreads();
    if(i % 3 < 2)
        sPMas[block_matId][i + 1][i] = sPMas[block_matId][i][i + 1];
    else
        sPMas[block_matId][i][i - 2] = sPMas[block_matId][i - 2][i];
    __syncthreads();

    for(int j = 0; j < (BANKSIZE * 3); j++)
    {
        int rowId = j / 3;
        int colId = i / 3;
        if(colId >= rowId)
        {
            int index = BANKSIZE * rowId - rowId * (rowId + 1) / 2 + colId;
            preMatrix[matId].M[index](j % 3, i % 3) = (float)sPMas[block_matId][j][i];
        }
    }
}

// ============================================================================
// Kernel: Build multi-level residual
// ============================================================================
__global__ void k_buildMultiLevelR(const double3* __restrict__ R,
                                   Eigen::Vector3f* __restrict__ multiLR,
                                   const int* __restrict__ goingNext,
                                   const int* __restrict__ prefixOrigin,
                                   const unsigned int* __restrict__ fineConnectMsk,
                                   const int* __restrict__ partId_map_real,
                                   int levelNum, int numbers)
{
    int pdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(pdx >= numbers) return;

    Eigen::Vector3f r;
    int idx = partId_map_real[pdx];

    if(idx >= 0)
    {
        r[0] = (float)R[idx].x;
        r[1] = (float)R[idx].y;
        r[2] = (float)R[idx].z;
    }
    else
    {
        r[0] = 0; r[1] = 0; r[2] = 0;
    }

    int laneId      = threadIdx.x % BANKSIZE;
    int localWarpId = threadIdx.x / BANKSIZE;
    int gwarpId     = pdx / BANKSIZE;

    multiLR[pdx] = r;

    __shared__ float c_sumResidual[DEFAULT_BLOCKSIZE * 3];
    __shared__ int   prefixSum[DEFAULT_WARPNUM];

    if(laneId == 0) prefixSum[localWarpId] = prefixOrigin[gwarpId];

    if(idx >= 0)
    {
        unsigned int connectMsk = fineConnectMsk[idx];

        if(prefixSum[localWarpId] == 1)
        {
            auto mask_val  = __activemask();
            int  warpId    = threadIdx.x & 0x1f;
            bool bBoundary = (laneId == 0) || (warpId == 0);

            unsigned int mark     = __ballot_sync(mask_val, bBoundary);
            mark                  = __brev(mark);
            int          clzlen   = __clz(mark << (warpId + 1));
            unsigned int interval = min(clzlen, 31 - warpId);

            for(int iter = 1; iter < BANKSIZE; iter <<= 1)
            {
                float tmpx = __shfl_down_sync(mask_val, r[0], iter);
                float tmpy = __shfl_down_sync(mask_val, r[1], iter);
                float tmpz = __shfl_down_sync(mask_val, r[2], iter);
                if(interval >= (unsigned int)iter)
                {
                    r[0] += tmpx; r[1] += tmpy; r[2] += tmpz;
                }
            }

            if(bBoundary)
            {
                int level = 0;
                while(level < levelNum - 1)
                {
                    level++;
                    idx = goingNext[idx];
                    atomicAdd(&(multiLR[idx][0]), r[0]);
                    atomicAdd(&(multiLR[idx][1]), r[1]);
                    atomicAdd(&(multiLR[idx][2]), r[2]);
                }
            }
            return;
        }
        else
        {
            int elected_lane = __ffs(connectMsk) - 1;

            c_sumResidual[threadIdx.x]                         = 0;
            c_sumResidual[threadIdx.x + DEFAULT_BLOCKSIZE]     = 0;
            c_sumResidual[threadIdx.x + 2 * DEFAULT_BLOCKSIZE] = 0;

            atomicAdd(c_sumResidual + localWarpId * BANKSIZE + elected_lane, r[0]);
            atomicAdd(c_sumResidual + localWarpId * BANKSIZE + elected_lane + DEFAULT_BLOCKSIZE, r[1]);
            atomicAdd(c_sumResidual + localWarpId * BANKSIZE + elected_lane + 2 * DEFAULT_BLOCKSIZE, r[2]);

            unsigned int electedPrefix = __popc(connectMsk & lanemask_lt(laneId));
            if(electedPrefix == 0)
            {
                int level = 0;
                while(level < levelNum - 1)
                {
                    level++;
                    idx = goingNext[idx];
                    atomicAdd(&(multiLR[idx][0]), c_sumResidual[threadIdx.x]);
                    atomicAdd(&(multiLR[idx][1]), c_sumResidual[threadIdx.x + DEFAULT_BLOCKSIZE]);
                    atomicAdd(&(multiLR[idx][2]), c_sumResidual[threadIdx.x + DEFAULT_BLOCKSIZE * 2]);
                }
            }
        }
    }
}

// ============================================================================
// Kernel: Schwarz local solve (block3 variant)
// ============================================================================
__global__ void k_schwarzLocalXSym_block3(const ClusterMatrixSymF* __restrict__ Pred,
                                          const Eigen::Vector3f* __restrict__ mR,
                                          float3* __restrict__ mZ,
                                          int number)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= number) return;

    int hessianSize = BANKSIZE * BANKSIZE;
    int Hid   = idx / hessianSize;
    int lvrid = (idx % hessianSize) / BANKSIZE;
    int lvcid = (idx % hessianSize) % BANKSIZE;

    int vrid = Hid * BANKSIZE + lvrid;
    int vcid = Hid * BANKSIZE + lvcid;

    Eigen::Vector3f rdata;

    __shared__ Eigen::Vector3f smR[BANKSIZE];

    if(threadIdx.x < BANKSIZE)
        smR[threadIdx.x] = mR[vcid];
    __syncthreads();

    if(vcid >= vrid)
    {
        int index = BANKSIZE * lvrid - lvrid * (lvrid + 1) / 2 + lvcid;
        rdata = Pred[Hid].M[index] * smR[lvcid];
    }
    else
    {
        int index = BANKSIZE * lvcid - lvcid * (lvcid + 1) / 2 + lvrid;
        rdata = Pred[Hid].M[index].transpose() * smR[lvcid];
    }

    int  warpId    = threadIdx.x & 0x1f;
    int  landidx   = threadIdx.x % BANKSIZE;
    bool bBoundary = (landidx == 0) || (warpId == 0);

    unsigned int mark     = __ballot_sync(0xffffffff, bBoundary);
    mark                  = __brev(mark);
    int          clzlen   = __clz(mark << (warpId + 1));
    unsigned int interval = min(clzlen, 31 - warpId);

    int maxSize = min(32, BANKSIZE);
    for(int iter = 1; iter < maxSize; iter <<= 1)
    {
        float tmpx = __shfl_down_sync(0xffffffff, rdata[0], iter);
        float tmpy = __shfl_down_sync(0xffffffff, rdata[1], iter);
        float tmpz = __shfl_down_sync(0xffffffff, rdata[2], iter);
        if(interval >= (unsigned int)iter)
        {
            rdata[0] += tmpx; rdata[1] += tmpy; rdata[2] += tmpz;
        }
    }

    if(bBoundary)
    {
        atomicAdd(&(mZ[vrid].x), rdata[0]);
        atomicAdd(&(mZ[vrid].y), rdata[1]);
        atomicAdd(&(mZ[vrid].z), rdata[2]);
    }
}

// ============================================================================
// Kernel: Collect final Z
// ============================================================================
__global__ void k_collectFinalZ(double3* __restrict__ Z,
                                const float3* __restrict__ d_multiLevelZ,
                                const LevelTable* __restrict__ coarseTable,
                                const int* __restrict__ real_map_partId,
                                int levelnum, int number)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= number) return;

    int    rdx = real_map_partId[idx];
    float3 cz;
    cz.x = d_multiLevelZ[rdx].x;
    cz.y = d_multiLevelZ[rdx].y;
    cz.z = d_multiLevelZ[rdx].z;

    LevelTable table   = coarseTable[idx];
    const int* tablePtr = table.index;

    for(int i = 1; i < levelnum; i++)
    {
        int now = tablePtr[i - 1];
        cz.x += d_multiLevelZ[now].x;
        cz.y += d_multiLevelZ[now].y;
        cz.z += d_multiLevelZ[now].z;
    }

    Z[idx].x = (double)cz.x;
    Z[idx].y = (double)cz.y;
    Z[idx].z = (double)cz.z;
}

// ============================================================================
// Host methods
// ============================================================================

void MASPreconditionerEngine::compute_num_levels(int vertNum)
{
    int nLevel  = 1;
    int levelSz = (vertNum + BANKSIZE - 1) / BANKSIZE * BANKSIZE;

    while(levelSz > BANKSIZE)
    {
        levelSz /= BANKSIZE;
        nLevel++;
        levelSz = (levelSz + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
    }
    nLevel++;
    m_levelnum = nLevel > MAX_LEVELS ? MAX_LEVELS : nLevel;
}

void MASPreconditionerEngine::init_neighbor(
    int                              vertNum,
    int                              totalNeighborNum,
    int                              partMapSize,
    const std::vector<unsigned int>& h_neighborList,
    const std::vector<unsigned int>& h_neighborStart,
    const std::vector<unsigned int>& h_neighborNum,
    const std::vector<int>&          h_partId_map_real,
    const std::vector<int>&          h_real_map_partId)
{
    if(vertNum < 1) return;

    int maxNodes = partMapSize > vertNum ? partMapSize : vertNum;
    compute_num_levels(maxNodes);

    m_totalMapNodes = partMapSize;
    m_totalNodes    = vertNum;

    // Allocate hierarchy buffers
    m_d_denseLevel.resize(vertNum);
    m_d_real_map_partId.resize(vertNum);
    m_d_coarseTable.resize(vertNum);
    m_d_coarseSpaceTables.resize(vertNum * m_levelnum);
    m_d_levelSize.resize(m_levelnum + 1);
    m_d_goingNext.resize(vertNum * m_levelnum);
    m_d_prefixOriginal.resize(vertNum);
    m_d_nextPrefix.resize(vertNum);
    m_d_nextPrefixSum.resize(vertNum);
    m_d_prefixSumOriginal.resize(vertNum);
    m_d_fineConnectMask.resize(vertNum);
    m_d_nextConnectMask.resize(vertNum);

    // Neighbor buffers
    m_neighborListSize = totalNeighborNum;
    m_d_neighborList.resize(totalNeighborNum);
    m_d_neighborStart.resize(vertNum);
    m_d_neighborNum.resize(vertNum);
    m_d_neighborListInit.resize(totalNeighborNum);
    m_d_neighborNumInit.resize(vertNum);

    // Partition mappings
    m_d_partId_map_real.resize(partMapSize);

    // Upload host data
    m_d_neighborListInit.view().copy_from(h_neighborList.data());
    m_d_neighborStart.view().copy_from(h_neighborStart.data());
    m_d_neighborNumInit.view().copy_from(h_neighborNum.data());
    m_d_partId_map_real.view().copy_from(h_partId_map_real.data());
    m_d_real_map_partId.view().copy_from(h_real_map_partId.data());
}

void MASPreconditionerEngine::init_matrix()
{
    if(m_totalNodes < 1) return;

    // Copy neighbor data for first use
    m_d_neighborList.view().copy_from(m_d_neighborListInit.view());
    m_d_neighborNum.view().copy_from(m_d_neighborNumInit.view());

    int totalCluster = (int)(reorder_realtime(0) * 1.05);

    int numClusterBlocks = totalCluster / BANKSIZE;
    m_d_inverseMatMas.resize(numClusterBlocks);
    m_d_precondMatMas.resize(numClusterBlocks);
    m_d_multiLevelR.resize(totalCluster);
    m_d_multiLevelZ.resize(totalCluster);

    m_initialized = true;
}

int MASPreconditionerEngine::reorder_realtime(int cpNum)
{
    m_d_levelSize.fill(Int2{0, 0});

    build_connect_mask_L0();
    prepare_prefix_sum_L0();
    build_level1();

    for(int level = 1; level < m_levelnum; level++)
    {
        m_d_nextConnectMask.fill(0);
        build_connect_mask_Lx(level);

        // Copy level size to host
        cudaMemcpy(&m_h_clevelSize,
                   m_d_levelSize.data() + level,
                   sizeof(Int2),
                   cudaMemcpyDeviceToHost);

        next_level_cluster(level);
        prefix_sum_Lx(level);
        compute_next_level(level);
    }

    cudaMemcpy(&m_h_clevelSize,
               m_d_levelSize.data() + m_levelnum,
               sizeof(Int2),
               cudaMemcpyDeviceToHost);

    m_totalNumberClusters = m_h_clevelSize.y;
    aggregation_kernel();

    return m_totalNumberClusters;
}

void MASPreconditionerEngine::build_connect_mask_L0()
{
    int number = m_totalMapNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_buildCML0<<<numBlocks, blockSize>>>(
        m_d_neighborStart.data(),
        m_d_neighborNum.data(),
        m_d_neighborList.data(),
        m_d_fineConnectMask.data(),
        m_d_partId_map_real.data(),
        m_d_real_map_partId.data(),
        number);
}

void MASPreconditionerEngine::prepare_prefix_sum_L0()
{
    int number = m_totalMapNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_preparePrefixSumL0<<<numBlocks, blockSize>>>(
        m_d_prefixOriginal.data(),
        m_d_fineConnectMask.data(),
        m_d_partId_map_real.data(),
        number);
}

void MASPreconditionerEngine::build_level1()
{
    int number = m_totalMapNodes;
    if(number < 1) return;
    int blockSize = BANKSIZE * BANKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    int warpNum   = (number + BANKSIZE - 1) / BANKSIZE;

    thrust::exclusive_scan(thrust::device_ptr<int>(m_d_prefixOriginal.data()),
                           thrust::device_ptr<int>(m_d_prefixOriginal.data()) + warpNum,
                           thrust::device_ptr<int>(m_d_prefixSumOriginal.data()));

    k_buildLevel1<<<numBlocks, blockSize>>>(
        m_d_levelSize.data(),
        m_d_coarseSpaceTables.data(),
        m_d_goingNext.data(),
        m_d_fineConnectMask.data(),
        m_d_prefixSumOriginal.data(),
        m_d_prefixOriginal.data(),
        m_d_partId_map_real.data(),
        number);
}

void MASPreconditionerEngine::build_connect_mask_Lx(int level)
{
    int number = m_totalMapNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_buildConnectMaskLx<<<numBlocks, blockSize>>>(
        m_d_neighborStart.data(),
        m_d_neighborNum.data(),
        m_d_neighborList.data(),
        m_d_coarseSpaceTables.data(),
        m_d_nextConnectMask.data(),
        m_d_fineConnectMask.data(),
        level,
        m_d_partId_map_real.data(),
        m_totalNodes,
        number);
}

void MASPreconditionerEngine::next_level_cluster(int level)
{
    int number = m_h_clevelSize.x;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_nextLevelCluster<<<numBlocks, blockSize>>>(
        m_d_nextConnectMask.data(), m_d_nextPrefix.data(), number);
}

void MASPreconditionerEngine::prefix_sum_Lx(int level)
{
    int number     = m_h_clevelSize.x;
    if(number < 1) return;
    int levelBegin = m_h_clevelSize.y;
    int blockSize  = BANKSIZE * BANKSIZE;
    int numBlocks  = (number + blockSize - 1) / blockSize;
    int warpNum    = (number + BANKSIZE - 1) / BANKSIZE;

    thrust::exclusive_scan(thrust::device_ptr<unsigned int>(m_d_nextPrefix.data()),
                           thrust::device_ptr<unsigned int>(m_d_nextPrefix.data()) + warpNum,
                           thrust::device_ptr<unsigned int>(m_d_nextPrefixSum.data()));

    k_prefixSumLx<<<numBlocks, blockSize>>>(
        m_d_levelSize.data(),
        m_d_nextPrefix.data(),
        m_d_nextPrefixSum.data(),
        m_d_nextConnectMask.data(),
        m_d_goingNext.data(),
        level, levelBegin, number);
}

void MASPreconditionerEngine::compute_next_level(int level)
{
    int number = m_totalNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_computeNextLevel<<<numBlocks, blockSize>>>(
        m_d_coarseSpaceTables.data(), m_d_nextConnectMask.data(), level, number);
}

void MASPreconditionerEngine::aggregation_kernel()
{
    int number = m_totalNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_aggregationKernel<<<numBlocks, blockSize>>>(
        m_d_denseLevel.data(), m_d_coarseTable.data(),
        m_d_goingNext.data(), m_levelnum, number);
}

// ============================================================================
// Assemble preconditioner
// ============================================================================

void MASPreconditionerEngine::set_preconditioner(
    const Eigen::Matrix3d* d_triplet_values,
    const int*             d_row_ids,
    const int*             d_col_ids,
    const uint32_t*        d_indices,
    int                    dof_offset,
    int                    triplet_num,
    int                    cpNum)
{
    if(m_totalNodes < 1) return;

    // Reset neighbor data for this iteration
    m_d_neighborList.view().copy_from(m_d_neighborListInit.view());
    m_d_neighborNum.view().copy_from(m_d_neighborNumInit.view());

    reorder_realtime(cpNum);

    // Resize and zero out the cluster matrices
    int numClusterBlocks = m_totalNumberClusters / BANKSIZE;
    if(numClusterBlocks < 1) return;

    if(numClusterBlocks > (int)m_d_inverseMatMas.size())
    {
        m_d_inverseMatMas.resize(numClusterBlocks);
        m_d_precondMatMas.resize(numClusterBlocks);
    }

    // Ensure multi-level buffers are large enough
    if(m_totalNumberClusters > (int)m_d_multiLevelR.size())
    {
        m_d_multiLevelR.resize(m_totalNumberClusters);
        m_d_multiLevelZ.resize(m_totalNumberClusters);
    }

    cudaMemset(m_d_inverseMatMas.data(), 0, numClusterBlocks * sizeof(ClusterMatrixSym));

    prepare_hessian_bcoo(d_triplet_values, d_row_ids, d_col_ids, d_indices, dof_offset, triplet_num);

    // Invert cluster matrices
    int blockSize2 = 32 * 3;
    int number2    = m_totalNumberClusters * 3;
    if(number2 < 1) return;
    int numBlocks2 = (number2 + blockSize2 - 1) / blockSize2;
    k_invertClusterMatrix<<<numBlocks2, blockSize2>>>(
        m_d_precondMatMas.data(), m_d_inverseMatMas.data(), number2);
}

void MASPreconditionerEngine::prepare_hessian_bcoo(
    const Eigen::Matrix3d* d_triplet_values,
    const int*             d_row_ids,
    const int*             d_col_ids,
    const uint32_t*        d_indices,
    int                    dof_offset,
    int                    triplet_num)
{
    using namespace muda;

    // Scatter triplets into cluster matrices
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(
            triplet_num,
            [offset           = dof_offset,
             levelNum         = m_levelnum,
             goingNext        = m_d_goingNext.data(),
             invMatrix        = m_d_inverseMatMas.data(),
             real_map_partId  = m_d_real_map_partId.data(),
             indices          = d_indices,
             triplet_values   = d_triplet_values,
             row_ids          = d_row_ids,
             col_ids          = d_col_ids,
             totalNodes       = m_totalNodes] __device__(int I) mutable
            {
                int  index         = indices[I];
                auto vertRid_real  = row_ids[index];
                auto vertCid_real  = col_ids[index];
                auto H             = triplet_values[index];

                vertRid_real -= offset;
                vertCid_real -= offset;

                // Skip triplets outside the FEM DOF range
                if(vertRid_real < 0 || vertRid_real >= totalNodes
                   || vertCid_real < 0 || vertCid_real >= totalNodes)
                    return;

                int vertCid = real_map_partId[vertCid_real];
                int vertRid = real_map_partId[vertRid_real];
                int cPid    = vertCid / BANKSIZE;

                if(vertCid / BANKSIZE == vertRid / BANKSIZE)
                {
                    if(vertCid >= vertRid)
                    {
                        int bvRid = vertRid % BANKSIZE;
                        int bvCid = vertCid % BANKSIZE;
                        int idx   = BANKSIZE * bvRid - bvRid * (bvRid + 1) / 2 + bvCid;
                        invMatrix[cPid].M[idx] = H;
                    }
                    else
                    {
                        // Partition reordering flipped the order;
                        // store transpose in the symmetric upper-triangle position
                        int bvRid = vertCid % BANKSIZE;
                        int bvCid = vertRid % BANKSIZE;
                        int idx   = BANKSIZE * bvRid - bvRid * (bvRid + 1) / 2 + bvCid;
                        invMatrix[cPid].M[idx] = H.transpose();
                    }
                }
                else
                {
                    int level = 0;
                    while(level < levelNum - 1)
                    {
                        level++;
                        if(level == 1)
                        {
                            vertCid = goingNext[vertCid_real];
                            vertRid = goingNext[vertRid_real];
                        }
                        else
                        {
                            vertCid = goingNext[vertCid];
                            vertRid = goingNext[vertRid];
                        }
                        cPid = vertCid / BANKSIZE;

                        if(vertCid / BANKSIZE == vertRid / BANKSIZE)
                        {
                            if(vertCid >= vertRid)
                            {
                                int bvRid = vertRid % BANKSIZE;
                                int bvCid = vertCid % BANKSIZE;
                                int idx   = BANKSIZE * bvRid - bvRid * (bvRid + 1) / 2 + bvCid;
                                for(int i = 0; i < 3; i++)
                                    for(int j = 0; j < 3; j++)
                                    {
                                        atomicAdd(&(invMatrix[cPid].M[idx](i, j)), H(i, j));
                                        if(vertCid == vertRid)
                                            atomicAdd(&(invMatrix[cPid].M[idx](i, j)), H(j, i));
                                    }
                            }
                            else
                            {
                                int bvRid = vertRid % BANKSIZE;
                                int bvCid = vertCid % BANKSIZE;
                                int idx   = BANKSIZE * bvCid - bvCid * (bvCid + 1) / 2 + bvRid;
                                for(int i = 0; i < 3; i++)
                                    for(int j = 0; j < 3; j++)
                                        atomicAdd(&(invMatrix[cPid].M[idx](i, j)), H(j, i));
                            }
                        }
                    }
                }
            });

    // Scatter diagonal blocks to coarser levels
    int tripletNum = m_totalMapNodes * BANKSIZE;
    if(tripletNum < 1) return;
    int threadNum = BANKSIZE * BANKSIZE;
    int blockNum  = (tripletNum + threadNum - 1) / threadNum;

    ParallelFor(blockNum, threadNum)
        .file_line(__FILE__, __LINE__)
        .apply(
            tripletNum,
            [levelNum        = m_levelnum,
             goingNext       = m_d_goingNext.data(),
             invMatrix       = m_d_inverseMatMas.data(),
             partId_map_real = m_d_partId_map_real.data(),
             fineConnectMsk  = m_d_fineConnectMask.data(),
             prefix0         = m_d_prefixOriginal.data()] __device__(int idx) mutable
            {
                int HSIZE = BANKSIZE * BANKSIZE;
                int Hid   = idx / HSIZE;
                int LMRid = (idx % HSIZE) / BANKSIZE;
                int LMCid = (idx % HSIZE) % BANKSIZE;

                int MRid = Hid * BANKSIZE + LMRid;
                int MCid = Hid * BANKSIZE + LMCid;

                int rdx = partId_map_real[MRid];
                int cdx = partId_map_real[MCid];

                __shared__ int prefix;
                if(threadIdx.x == 0) prefix = prefix0[Hid];
                __syncthreads();

                Eigen::Matrix3d mat3;
                if(LMCid >= LMRid)
                {
                    int index = BANKSIZE * LMRid - LMRid * (LMRid + 1) / 2 + LMCid;
                    mat3 = invMatrix[Hid].M[index];
                }
                else
                {
                    int index = BANKSIZE * LMCid - LMCid * (LMCid + 1) / 2 + LMRid;
                    mat3 = invMatrix[Hid].M[index].transpose();
                }

                if((rdx >= 0) && (cdx >= 0))
                {
                    if(prefix == 1)
                    {
                        int  warpId    = threadIdx.x & 0x1f;
                        bool bBoundary = (warpId == 0) || (rdx < 0) || (cdx < 0);
                        unsigned int mark = __ballot_sync(0xffffffff, bBoundary);
                        mark = __brev(mark);
                        int clzlen       = __clz(mark << (warpId + 1));
                        unsigned int interval = min(clzlen, 31 - warpId);

                        for(int iter = 1; iter < 32; iter <<= 1)
                        {
                            Eigen::Matrix3d matTemp;
                            for(int i = 0; i < 3; i++)
                                for(int j = 0; j < 3; j++)
                                    matTemp(i, j) = __shfl_down_sync(0xffffffff, mat3(i, j), iter);
                            if(interval >= (unsigned int)iter)
                                mat3 = mat3 + matTemp;
                        }

                        if(bBoundary)
                        {
                            int level  = 0;
                            int nextId = goingNext[rdx];
                            while(level < levelNum - 1)
                            {
                                level++;
                                int cPid  = nextId / BANKSIZE;
                                int bvRid = nextId % BANKSIZE;
                                int bvCid = nextId % BANKSIZE;
                                int index = BANKSIZE * bvRid - bvRid * (bvRid + 1) / 2 + bvCid;
                                for(int i = 0; i < 3; i++)
                                    for(int j = 0; j < 3; j++)
                                        atomicAdd(&(invMatrix[cPid].M[index](i, j)), mat3(i, j));
                                nextId = goingNext[nextId];
                            }
                        }
                    }
                    else
                    {
                        int level = 0;
                        while(level < levelNum - 1)
                        {
                            level++;
                            rdx      = goingNext[rdx];
                            cdx      = goingNext[cdx];
                            int cPid = cdx / BANKSIZE;
                            if(rdx / BANKSIZE == cdx / BANKSIZE)
                            {
                                if(cdx >= rdx)
                                {
                                    int bvRid = rdx % BANKSIZE;
                                    int bvCid = cdx % BANKSIZE;
                                    int index = BANKSIZE * bvRid - bvRid * (bvRid + 1) / 2 + bvCid;
                                    for(int i = 0; i < 3; i++)
                                        for(int j = 0; j < 3; j++)
                                            atomicAdd(&(invMatrix[cPid].M[index](i, j)), mat3(i, j));
                                }
                            }
                        }
                    }
                }
            });
}

// ============================================================================
// Apply preconditioning
// ============================================================================

void MASPreconditionerEngine::build_multi_level_R(const double3* R)
{
    int number = m_totalMapNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_buildMultiLevelR<<<numBlocks, blockSize>>>(
        R,
        m_d_multiLevelR.data(),
        m_d_goingNext.data(),
        m_d_prefixOriginal.data(),
        m_d_fineConnectMask.data(),
        m_d_partId_map_real.data(),
        m_levelnum, number);
}

void MASPreconditionerEngine::schwarz_local_solve()
{
    int number = m_totalNumberClusters * BANKSIZE;
    if(number < 1) return;
    int blockSize = BANKSIZE * BANKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_schwarzLocalXSym_block3<<<numBlocks, blockSize>>>(
        m_d_precondMatMas.data(), m_d_multiLevelR.data(), m_d_multiLevelZ.data(), number);
}

void MASPreconditionerEngine::collect_final_Z(double3* Z)
{
    int number = m_totalNodes;
    if(number < 1) return;
    int blockSize = DEFAULT_BLOCKSIZE;
    int numBlocks = (number + blockSize - 1) / blockSize;
    k_collectFinalZ<<<numBlocks, blockSize>>>(
        Z, m_d_multiLevelZ.data(), m_d_coarseTable.data(),
        m_d_real_map_partId.data(), m_levelnum, number);
}

void MASPreconditionerEngine::apply(muda::CDenseVectorView<Float> r,
                                    muda::DenseVectorView<Float>  z)
{
    if(m_totalNodes < 1) return;

    // Ensure multi-level buffers cover all clusters
    if(m_totalNumberClusters > (int)m_d_multiLevelR.size())
    {
        m_d_multiLevelR.resize(m_totalNumberClusters);
        m_d_multiLevelZ.resize(m_totalNumberClusters);
    }

    // Zero coarse-level residuals (beyond fine nodes)
    if(m_totalNumberClusters > m_totalMapNodes)
    {
        cudaMemset(m_d_multiLevelR.data() + m_totalMapNodes,
                   0,
                   (m_totalNumberClusters - m_totalMapNodes) * sizeof(Eigen::Vector3f));
    }

    // Zero all solution levels
    cudaMemset(m_d_multiLevelZ.data(), 0, m_totalNumberClusters * sizeof(float3));

    // 1. Restrict residual down through levels
    build_multi_level_R((const double3*)r.data());

    // 2. Local block solve at each level
    schwarz_local_solve();

    // 3. Prolongate solution back up
    collect_final_Z((double3*)z.data());
}

}  // namespace uipc::backend::cuda
