#include "stacklessbvh.cuh"
#include <cuda_device/builtin.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/swap.h>
#include <thrust/sequence.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/functional.h>
#include <thrust/sort.h>
#include <thrust/fill.h>
#include <thrust/reduce.h>
#include <thrust/execution_policy.h>

#define MAX_CD_NUM_PER_VERT 64
namespace culbvh
{
using uint                   = uint32_t;
using ullint                 = unsigned long long int;
int const K_THREADS          = 256;
int const K_REDUCTION_LAYER  = 5;
int const K_REDUCTION_NUM    = 1 << K_REDUCTION_LAYER;
int const K_REDUCTION_MODULO = K_REDUCTION_NUM - 1;

int const    aabbBits  = 15;
int const    aabbRes   = (1 << aabbBits) - 2;
int const    indexBits = 64 - 3 * aabbBits;
int const    offset3   = aabbBits * 3;
int const    offset2   = aabbBits * 2;
int const    offset1   = aabbBits * 1;
ullint const indexMask = 0xFFFFFFFFFFFFFFFFu << offset3;
uint const   aabbMask  = 0xFFFFFFFFu >> (32 - aabbBits);
uint const   MaxIndex  = 0xFFFFFFFFFFFFFFFFu >> offset3;

namespace LBVHKernels
{

    struct intAABB
    {
        int3                                     _min, _max;
        __host__ __device__ __forceinline__ void convertFrom(const aabb& other,
                                                             float3&     origin,
                                                             float3&     delta)
        {
            _min.x = static_cast<int>((other.min.x - origin.x) / delta.x);
            _min.y = static_cast<int>((other.min.y - origin.y) / delta.y);
            _min.z = static_cast<int>((other.min.z - origin.z) / delta.z);
            _max.x = static_cast<int>(ceilf((other.max.x - origin.x) / delta.x));
            _max.y = static_cast<int>(ceilf((other.max.y - origin.y) / delta.y));
            _max.z = static_cast<int>(ceilf((other.max.z - origin.z) / delta.z));
        }
    };

    template <typename T>
    __device__ __host__ inline T __mm_min(T a, T b)
    {
        return a > b ? b : a;
    }

    template <typename T>
    __device__ __host__ inline T __mm_max(T a, T b)
    {
        return a > b ? a : b;
    }

    __device__ __forceinline__ float atomicMinf(float* addr, float value)
    {
        float old;
        old = (value >= 0) ?
                  __int_as_float(atomicMin((int*)addr, __float_as_int(value))) :
                  __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));
        return old;
    }

    __device__ __forceinline__ float atomicMaxf(float* addr, float value)
    {
        float old;
        old = (value >= 0) ?
                  __int_as_float(atomicMax((int*)addr, __float_as_int(value))) :
                  __uint_as_float(atomicMin((unsigned int*)addr, __float_as_uint(value)));
        return old;
    }

    __device__ uint expandBits(uint v)
    {  ///< Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
        v = (v * 0x00010001u) & 0xFF0000FFu;
        v = (v * 0x00000101u) & 0x0F00F00Fu;
        v = (v * 0x00000011u) & 0xC30C30C3u;
        v = (v * 0x00000005u) & 0x49249249u;
        return v;
    }

    __device__ uint morton3D(float x, float y, float z)
    {  ///< Calculates a 30-bit Morton code for the given 3D point located within the unit cube [0,1].
        x       = ::fmin(::fmax(x * 1024.0f, 0.0f), 1023.0f);
        y       = ::fmin(::fmax(y * 1024.0f, 0.0f), 1023.0f);
        z       = ::fmin(::fmax(z * 1024.0f, 0.0f), 1023.0f);
        uint xx = expandBits((uint)x);
        uint yy = expandBits((uint)y);
        uint zz = expandBits((uint)z);
        return (xx * 4 + yy * 2 + zz);
    }

    // Custom comparison for int3 based on lexicographical ordering
    __device__ inline bool lessThan(const int3& a, const int3& b)
    {
        if(a.x != b.x)
            return a.x < b.x;
        if(a.y != b.y)
            return a.y < b.y;
        return a.z < b.z;
    }

    __global__ void calcMaxBVFromBox(int size, const aabb* box, aabb* _bv)
    {
        int idx     = blockIdx.x * blockDim.x + threadIdx.x;
        int warpTid = threadIdx.x % 32;
        int warpId  = (threadIdx.x >> 5);
        int warpNum;
        if(idx >= size)
            return;
        if(idx == 0)
        {
            _bv[0] = aabb();
        }

        __shared__ aabb aabbData[K_THREADS >> 5];

        aabb temp = box[idx];
        __syncthreads();

        for(int i = 1; i < 32; i = (i << 1))
        {
            float tempMinX = __shfl_down_sync(0xffffffff, temp.min.x, i);
            float tempMinY = __shfl_down_sync(0xffffffff, temp.min.y, i);
            float tempMinZ = __shfl_down_sync(0xffffffff, temp.min.z, i);
            float tempMaxX = __shfl_down_sync(0xffffffff, temp.max.x, i);
            float tempMaxY = __shfl_down_sync(0xffffffff, temp.max.y, i);
            float tempMaxZ = __shfl_down_sync(0xffffffff, temp.max.z, i);
            temp.min.x     = __mm_min(temp.min.x, tempMinX);
            temp.min.y     = __mm_min(temp.min.y, tempMinY);
            temp.min.z     = __mm_min(temp.min.z, tempMinZ);
            temp.max.x     = __mm_max(temp.max.x, tempMaxX);
            temp.max.y     = __mm_max(temp.max.y, tempMaxY);
            temp.max.z     = __mm_max(temp.max.z, tempMaxZ);
        }

        if(blockIdx.x == gridDim.x - 1)
        {
            //tidNum = numbers - idof;
            warpNum = ((size - blockIdx.x * blockDim.x + 31) >> 5);
        }
        else
        {
            warpNum = ((blockDim.x) >> 5);
        }

        if(warpTid == 0)
        {
            aabbData[warpId] = temp;
        }
        __syncthreads();
        if(threadIdx.x >= warpNum)
            return;

        if(warpNum > 1)
        {
            //	tidNum = warpNum;
            temp = aabbData[threadIdx.x];

            //	warpNum = ((tidNum + 31) >> 5);
            for(int i = 1; i < warpNum; i = (i << 1))
            {
                float tempMinX = __shfl_down_sync(0xffffffff, temp.min.x, i);
                float tempMinY = __shfl_down_sync(0xffffffff, temp.min.y, i);
                float tempMinZ = __shfl_down_sync(0xffffffff, temp.min.z, i);
                float tempMaxX = __shfl_down_sync(0xffffffff, temp.max.x, i);
                float tempMaxY = __shfl_down_sync(0xffffffff, temp.max.y, i);
                float tempMaxZ = __shfl_down_sync(0xffffffff, temp.max.z, i);
                temp.min.x     = __mm_min(temp.min.x, tempMinX);
                temp.min.y     = __mm_min(temp.min.y, tempMinY);
                temp.min.z     = __mm_min(temp.min.z, tempMinZ);
                temp.max.x     = __mm_max(temp.max.x, tempMaxX);
                temp.max.y     = __mm_max(temp.max.y, tempMaxY);
                temp.max.z     = __mm_max(temp.max.z, tempMaxZ);
            }
        }

        if(threadIdx.x == 0)
        {
            float* ptrLowerBound = reinterpret_cast<float*>(&(_bv[0].min));
            float* ptrUpperBound = reinterpret_cast<float*>(&(_bv[0].max));
            atomicMinf(ptrLowerBound, temp.min.x);
            atomicMinf(ptrLowerBound + 1, temp.min.y);
            atomicMinf(ptrLowerBound + 2, temp.min.z);
            atomicMaxf(ptrUpperBound, temp.max.x);
            atomicMaxf(ptrUpperBound + 1, temp.max.y);
            atomicMaxf(ptrUpperBound + 2, temp.max.z);
        }
    }


    __global__ void calcMCsFromBox(int size, const aabb* box, aabb* scene, uint* codes)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;
        aabb bv = box[idx];

        const float3 c      = bv.center();
        const float3 offset = c - scene[0].min;
        codes[idx]          = morton3D(offset.x / scene[0].width(),
                              offset.y / scene[0].height(),
                              offset.z / scene[0].depth());
    }

    /// incoherent access, thus poor performance
    __global__ void calcInverseMapping(int size, int* map, int* invMap)
    {
        int idx = blockDim.x * blockIdx.x + threadIdx.x;
        if(idx >= size)
            return;
        invMap[map[idx]] = idx;
    }

    __global__ void buildPrimitivesFromBox(
        int size, int* _primIdx, aabb* _primBox, int* _primMap, const aabb* box)
    {  ///< update idx-th _bxs to idx-th leaf
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;
        int  newIdx      = _primMap[idx];
        aabb bv          = box[idx];
        _primIdx[newIdx] = idx;
        _primBox[newIdx] = bv;
    }

    __global__ void calcExtNodeSplitMetrics(int extsize, const uint32_t* _codes, int* _metrics)
    {
        int idx = blockDim.x * blockIdx.x + threadIdx.x;
        if(idx >= extsize)
            return;
        _metrics[idx] =
            idx != extsize - 1 ? 32 - __clz(_codes[idx] ^ _codes[idx + 1]) : 33;
    }

    __global__ void buildIntNodes(int       size,
                                  uint32_t* _depths,
                                  int*      _lvs_lca,
                                  int*      _lvs_metric,
                                  uint32_t* _lvs_par,
                                  uint32_t* _lvs_mark,
                                  aabb*     _lvs_box,
                                  int*      _tks_rc,
                                  int*      _tks_lc,
                                  int*      _tks_range_y,
                                  int*      _tks_range_x,
                                  uint32_t* _tks_mark,
                                  aabb*     _tks_box,
                                  uint32_t* _flag,
                                  int*      _tks_par)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;
        //_tks_range_x[idx] = -1;
        //__syncthreads();


        _lvs_lca[idx] = -1, _depths[idx] = 0;
        int  l = idx - 1, r = idx;  ///< (l, r]
        bool mark;
        if(l >= 0)
            mark = _lvs_metric[l] < _lvs_metric[r];  //determine direction
        else
            mark = false;
        int cur = mark ? l : r;

        //if (cur == 254)printf("%d  %d  %d  %d  %d\n", idx, mark, _lvs_metric[l], _lvs_metric[r], cur);
        _lvs_par[idx] = cur;
        if(mark)
            _tks_rc[cur] = idx, _tks_range_y[cur]                 = idx,
            atomicOr(&_tks_mark[cur], 0x00000002), _lvs_mark[idx] = 0x00000007;
        else
            _tks_lc[cur] = idx, _tks_range_x[cur]                 = idx,
            atomicOr(&_tks_mark[cur], 0x00000001), _lvs_mark[idx] = 0x00000003;

        //__threadfence();
        while(atomicAdd(&_flag[cur], 1) == 1)
        {
            //_tks.update(cur, _lvs);	/// Update
            //_tks.refit(cur, _lvs);	/// Refit
            int      chl       = _tks_lc[cur];
            int      chr       = _tks_rc[cur];
            uint32_t temp_mark = _tks_mark[cur];
            if(temp_mark & 1)
            {
                _tks_box[cur] = _lvs_box[chl];
            }
            else
            {
                _tks_box[cur] = _tks_box[chl];
            }
            if(temp_mark & 2)
            {
                _tks_box[cur].absorb(_lvs_box[chr]);
            }
            else
            {
                _tks_box[cur].absorb(_tks_box[chr]);
            }

            _tks_mark[cur] &= 0x00000007;
            //if (idx < 10) printf("%d   %d\n", idx, _tks_mark[cur]);
            //if (_tks_range_x[cur] == 0) printf("cur:%d  %d  %d   %d   %d\n", cur, _tks_range_x[252], _tks_range_y[252], _tks_lc[252], _tks_rc[252]);
            l = _tks_range_x[cur] - 1, r = _tks_range_y[cur];
            _lvs_lca[l + 1] = cur /*, _tks.rcd(cur) = ++_lvs.rcl(r)*/, _depths[l + 1]++;
            if(l >= 0)
                mark = _lvs_metric[l] < _lvs_metric[r];  ///< true when right child, false otherwise
            else
                mark = false;

            if(l + 1 == 0 && r == size - 1)
            {
                _tks_par[cur] = -1;
                _tks_mark[cur] &= 0xFFFFFFFB;
                break;
            }

            int par       = mark ? l : r;
            _tks_par[cur] = par;
            if(mark)
                _tks_rc[par] = cur, _tks_range_y[par] = r,
                atomicAnd(&_tks_mark[par], 0xFFFFFFFD), _tks_mark[cur] |= 0x00000004;
            else
                _tks_lc[par] = cur, _tks_range_x[par] = l + 1,
                atomicAnd(&_tks_mark[par], 0xFFFFFFFE), _tks_mark[cur] &= 0xFFFFFFFB;
            __threadfence();
            cur = par;
        }
    }

    __global__ void calcIntNodeOrders(
        int size, int* _tks_lc, int* _lcas, uint32_t* _depths, uint32_t* _offsets, int* _tkMap)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;
        //for (; idx < size; idx += gridDim.x * blockDim.x) {
        int node = _lcas[idx], depth = _depths[idx], id = _offsets[idx];
        //if (node == 874)printf("%d\n", idx);
        if(node != -1)
        {
            for(; depth--; node = _tks_lc[node])
            {
                _tkMap[node] = id++;
            }
        }
        //}
    }

    __global__ void updateBvhExtNodeLinks(int size, const int* _mapTable, int* _lcas, uint* _pars)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;
        int ori;
        _pars[idx] = _mapTable[_pars[idx]];
        if((ori = _lcas[idx]) != -1)
            _lcas[idx] = _mapTable[ori] << 1;
        else
            _lcas[idx] = idx << 1 | 1;
    }

    __global__ void setEscape(int* _lcas, int index)
    {
        _lcas[index] = -1;
    }

    __global__ void reorderNode(int            intSize,
                                const int*     _tkMap,
                                int*           _lvs_lca,
                                aabb*          _lvs_box,
                                int*           _unorderedTks_lc,
                                uint32_t*      _unorderedTks_mark,
                                int*           _unorderedTks_rangey,
                                aabb*          _unorderedTks_box,
                                stacklessnode* _nodes)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= intSize + 1)
            return;

        stacklessnode Node;
        Node.lc    = -1;
        int escape = _lvs_lca[idx + 1];

        if(escape == -1)
        {
            Node.escape = -1;
        }
        else
        {
            int bLeaf = escape & 1;
            escape >>= 1;
            Node.escape = escape + (bLeaf ? intSize : 0);
        }
        Node.bound = _lvs_box[idx];


        _nodes[idx + intSize] = Node;

        if(idx >= intSize)
            return;

        stacklessnode internalNode;
        int           newId = _tkMap[idx];
        uint32_t      mark  = _unorderedTks_mark[idx];

        internalNode.lc    = mark & 1 ? _unorderedTks_lc[idx] + intSize :
                                        _tkMap[_unorderedTks_lc[idx]];
        internalNode.bound = _unorderedTks_box[idx];

        int internalEscape = _lvs_lca[_unorderedTks_rangey[idx] + 1];

        if(internalEscape == -1)
        {
            internalNode.escape = -1;
        }
        else
        {
            int bLeaf = internalEscape & 1;
            internalEscape >>= 1;
            internalNode.escape = internalEscape + (bLeaf ? intSize : 0);
        }
        _nodes[newId] = internalNode;
    }

    __device__ __host__ __forceinline__ void quantilizeAABB(ulonglong2& qaabb,
                                                            aabb&       box,
                                                            float3&     origin,
                                                            float3&     delta)
    {
        qaabb.x |= static_cast<ullint>((box.min.x - origin.x) / delta.x) << offset2;
        qaabb.x |= static_cast<ullint>((box.min.y - origin.y) / delta.y) << offset1;
        qaabb.x |= static_cast<ullint>((box.min.z - origin.z) / delta.z);

        qaabb.y |= static_cast<ullint>(ceilf((box.max.x - origin.x) / delta.x)) << offset2;
        qaabb.y |= static_cast<ullint>(ceilf((box.max.y - origin.y) / delta.y)) << offset1;
        qaabb.y |= static_cast<ullint>(ceilf((box.max.z - origin.z) / delta.z));
    }

    __global__ void reorderQuantilizedNode(int         intSize,
                                           const int*  _tkMap,
                                           int*        _lvs_lca,
                                           aabb*       _lvs_box,
                                           int*        _unorderedTks_lc,
                                           uint*       _unorderedTks_mark,
                                           int*        _unorderedTks_rangey,
                                           aabb*       _unorderedTks_box,
                                           aabb*       _scene_box,
                                           ulonglong2* _nodes)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= intSize + 1)
            return;
        auto origin = _scene_box[0].min;
        auto delta  = _scene_box[0].max - origin;

        delta /= ((1 << aabbBits) - 2);

        ulonglong2 node{0, 0};
        quantilizeAABB(node, _lvs_box[idx], origin, delta);


        node.x |= indexMask;
        int escape = _lvs_lca[idx + 1];

        if(escape == -1)
        {
            node.y |= indexMask;
        }
        else
        {
            int bLeaf = escape & 1;
            escape >>= 1;
            escape += (bLeaf ? intSize : 0);
            node.y |= *reinterpret_cast<ullint*>(&escape) << offset3;
        }

        _nodes[idx + intSize] = node;

        if(idx >= intSize)
            return;

        int  newId = _tkMap[idx];
        uint mark  = _unorderedTks_mark[idx];

        ulonglong2 internalNode{0, 0};

        quantilizeAABB(internalNode, _unorderedTks_box[idx], origin, delta);

        int lc = mark & 1 ? _unorderedTks_lc[idx] + intSize :
                            _tkMap[_unorderedTks_lc[idx]];
        internalNode.x |= ((ullint)lc << offset3);

        int internalEscape = _lvs_lca[_unorderedTks_rangey[idx] + 1];

        if(internalEscape == -1)
        {
            internalNode.y |= indexMask;
        }
        else
        {
            int bLeaf = internalEscape & 1;
            internalEscape >>= 1;
            internalEscape += (bLeaf ? intSize : 0);
            internalNode.y |= *reinterpret_cast<ullint*>(&internalEscape) << offset3;
        }

        _nodes[newId] = internalNode;
    }
    __device__ __host__ __forceinline__ bool overlapsLonglong2int(const ulonglong2& a,
                                                                  const intAABB& b)
    {
        int temp_a, temp_b;
        temp_a = (a.x >> offset2) & aabbMask;
        temp_b = b._max.x;
        if(temp_a > temp_b)
            return false;

        temp_a = (a.y >> offset2) & aabbMask;
        temp_b = b._min.x;
        if(temp_a < temp_b)
            return false;

        temp_a = (a.x >> offset1) & aabbMask;
        temp_b = b._max.y;
        if(temp_a > temp_b)
            return false;

        temp_a = (a.y >> offset1) & aabbMask;
        temp_b = b._min.y;
        if(temp_a < temp_b)
            return false;

        temp_a = a.x & aabbMask;
        temp_b = b._max.z;
        if(temp_a > temp_b)
            return false;

        temp_a = a.y & aabbMask;
        temp_b = b._min.z;
        if(temp_a < temp_b)
            return false;

        return true;
    }

#define MAX_RES_PER_BLOCK 1024
    __global__ void quantilizedStacklessCDSharedSelf(uint              Size,
                                                     const aabb*       _box,
                                                     const int         intSize,
                                                     const int*        _lvs_idx,
                                                     const aabb*       scene,
                                                     const ulonglong2* _nodes,
                                                     int*      resCounter,
                                                     int2*     res,
                                                     const int maxRes)
    {
        int  tid    = blockIdx.x * blockDim.x + threadIdx.x;
        bool active = tid < Size;
        auto origin = scene[0].min;
        auto delta  = scene[0].max - origin;
        delta /= aabbRes;
        int     idx;
        intAABB bv;
        if(active)
        {
            idx = _lvs_idx[tid];
            bv.convertFrom(_box[idx], origin, delta);
        }

        __shared__ int2 sharedRes[MAX_RES_PER_BLOCK];
        __shared__ int sharedCounter;  // How many results are cached in shared memory
        __shared__ int sharedGlobalIdx;  // Where to write in global memory
        if(threadIdx.x == 0)
            sharedCounter = 0;

        int  st = 0;
        uint lc;
        uint escape;
        while(true)
        {
            __syncthreads();
            if(active)
            {
                while(st != MaxIndex)
                {
                    ulonglong2 node = _nodes[st];
                    lc              = node.x >> offset3;
                    escape          = node.y >> offset3;
                    if(overlapsLonglong2int(node, bv))
                    {
                        if(lc == MaxIndex)
                        {
                            if(tid < st - intSize)
                            {
                                int sIdx = atomicAdd(&sharedCounter, 1);
                                if(sIdx >= MAX_RES_PER_BLOCK)
                                {
                                    break;
                                }

                                sharedRes[sIdx] = int2{idx, _lvs_idx[st - intSize]};
                            }
                            st = escape;
                        }
                        else
                        {
                            st = lc;
                        }
                    }
                    else
                    {
                        st = escape;
                    }
                }
            }
            // Flush whatever we have
            __syncthreads();
            int totalRes = ::min(sharedCounter, MAX_RES_PER_BLOCK);

            if(threadIdx.x == 0)
            {
                sharedGlobalIdx = atomicAdd(resCounter, totalRes);
            }

            __syncthreads();

            // Make sure we dont write out of bounds
            const int globalIdx = sharedGlobalIdx;

            if(globalIdx >= maxRes || !totalRes)
                return;  // Out of memory for results.
            if(threadIdx.x == 0)
                sharedCounter = 0;
            //
            //// If we got here with a half full buffer, we are done.
            bool done = totalRes < MAX_RES_PER_BLOCK;
            // If we are about to run out of memory, we are done.
            if(totalRes > maxRes - globalIdx)
            {
                totalRes = maxRes - globalIdx;
                done     = true;
            }

            // Copy full blocks
            int fullBlocks = (totalRes - 1) / (int)blockDim.x;
            for(int i = 0; i < fullBlocks; i++)
            {
                int offset              = i * blockDim.x + threadIdx.x;
                res[globalIdx + offset] = sharedRes[offset];
            }

            // Copy the rest
            int offset = fullBlocks * blockDim.x + threadIdx.x;
            if(offset < totalRes)
                res[globalIdx + offset] = sharedRes[offset];

            // Break if every thread is done.
            if(done)
                break;
        }
    }

    __global__ void quantilizedStacklessCDSharedOther(uint        Size,
                                                      const aabb* _box,
                                                      const int*  sortedIdx,
                                                      const int   intSize,
                                                      const int*  _lvs_idx,
                                                      const aabb* scene,
                                                      const ulonglong2* _nodes,
                                                      int*      resCounter,
                                                      int2*     res,
                                                      const int maxRes)
    {
        int  tid    = blockIdx.x * blockDim.x + threadIdx.x;
        bool active = tid < Size;
        auto origin = scene[0].min;
        auto delta  = scene[0].max - origin;

        delta /= aabbRes;
        int     idx;
        intAABB bv;
        if(active)
        {
            idx = sortedIdx[tid];
            bv.convertFrom(_box[idx], origin, delta);
        }

        __shared__ int2 sharedRes[MAX_RES_PER_BLOCK];
        __shared__ int sharedCounter;  // How many results are cached in shared memory
        __shared__ int sharedGlobalIdx;  // Where to write in global memory
        if(threadIdx.x == 0)
            sharedCounter = 0;

        int  st = 0;
        uint lc;
        uint escape;
        while(true)
        {
            __syncthreads();
            if(active)
            {
                while(st != MaxIndex)
                {
                    ulonglong2 node = _nodes[st];
                    lc              = node.x >> offset3;
                    escape          = node.y >> offset3;
                    if(overlapsLonglong2int(node, bv))
                    {
                        if(lc == MaxIndex)
                        {
                            int sIdx = atomicAdd(&sharedCounter, 1);
                            if(sIdx >= MAX_RES_PER_BLOCK)
                            {
                                break;
                            }
                            sharedRes[sIdx] = int2{idx, _lvs_idx[st - intSize]};

                            st = escape;
                        }
                        else
                        {
                            st = lc;
                        }
                    }
                    else
                    {
                        st = escape;
                    }
                }
            }
            // Flush whatever we have
            __syncthreads();
            int totalRes = ::min(sharedCounter, MAX_RES_PER_BLOCK);

            if(threadIdx.x == 0)
            {
                sharedGlobalIdx = atomicAdd(resCounter, totalRes);
            }

            __syncthreads();

            // Make sure we dont write out of bounds
            const int globalIdx = sharedGlobalIdx;

            if(globalIdx >= maxRes || !totalRes)
                return;  // Out of memory for results.
            if(threadIdx.x == 0)
                sharedCounter = 0;
            //
            //// If we got here with a half full buffer, we are done.
            bool done = totalRes < MAX_RES_PER_BLOCK;
            // If we are about to run out of memory, we are done.
            if(totalRes > maxRes - globalIdx)
            {
                totalRes = maxRes - globalIdx;
                done     = true;
            }

            // Copy full blocks
            int fullBlocks = (totalRes - 1) / (int)blockDim.x;
            for(int i = 0; i < fullBlocks; i++)
            {
                int offset              = i * blockDim.x + threadIdx.x;
                res[globalIdx + offset] = sharedRes[offset];
            }

            // Copy the rest
            int offset = fullBlocks * blockDim.x + threadIdx.x;
            if(offset < totalRes)
                res[globalIdx + offset] = sharedRes[offset];

            // Break if every thread is done.
            if(done)
                break;
        }
    }

    __global__ void StacklessCDSharedSelf(uint                 Size,
                                          const aabb*          _box,
                                          const int            intSize,
                                          const int*           _lvs_idx,
                                          const stacklessnode* _nodes,
                                          int*                 resCounter,
                                          int2*                res,
                                          const int            maxRes)
    {
        int  tid    = blockIdx.x * blockDim.x + threadIdx.x;
        bool active = tid < Size;
        int  idx;
        aabb bv;
        if(active)
        {
            idx = _lvs_idx[tid];
            bv  = _box[idx];
        }

        __shared__ int2 sharedRes[MAX_RES_PER_BLOCK];
        __shared__ int sharedCounter;  // How many results are cached in shared memory
        __shared__ int sharedGlobalIdx;  // Where to write in global memory
        if(threadIdx.x == 0)
            sharedCounter = 0;

        int           st = 0;
        stacklessnode node;
        while(true)
        {
            __syncthreads();
            if(active)
            {
                while(st != -1)
                {

                    node.lc     = _nodes[st].lc;
                    node.escape = _nodes[st].escape;
                    node.bound  = _nodes[st].bound;
                    //node = _nodes[st];
                    if(node.bound.intersects(bv))
                    {
                        if(node.lc == -1)
                        {
                            if(tid < st - intSize)
                            {
                                int sIdx = atomicAdd(&sharedCounter, 1);
                                if(sIdx >= MAX_RES_PER_BLOCK)
                                {
                                    break;
                                }

                                sharedRes[sIdx] = int2{idx, _lvs_idx[st - intSize]};
                            }
                            st = node.escape;
                        }
                        else
                        {
                            st = node.lc;
                        }
                    }
                    else
                    {
                        st = node.escape;
                    }
                }
            }
            // Flush whatever we have
            __syncthreads();
            int totalRes = ::min(sharedCounter, MAX_RES_PER_BLOCK);

            if(threadIdx.x == 0)
            {
                sharedGlobalIdx = atomicAdd(resCounter, totalRes);
            }

            __syncthreads();

            // Make sure we dont write out of bounds
            const int globalIdx = sharedGlobalIdx;

            if(globalIdx >= maxRes || !totalRes)
                return;  // Out of memory for results.
            if(threadIdx.x == 0)
                sharedCounter = 0;
            //
            //// If we got here with a half full buffer, we are done.
            bool done = totalRes < MAX_RES_PER_BLOCK;
            // If we are about to run out of memory, we are done.
            if(totalRes > maxRes - globalIdx)
            {
                totalRes = maxRes - globalIdx;
                done     = true;
            }

            // Copy full blocks
            int fullBlocks = (totalRes - 1) / (int)blockDim.x;
            for(int i = 0; i < fullBlocks; i++)
            {
                int offset              = i * blockDim.x + threadIdx.x;
                res[globalIdx + offset] = sharedRes[offset];
            }

            // Copy the rest
            int offset = fullBlocks * blockDim.x + threadIdx.x;
            if(offset < totalRes)
                res[globalIdx + offset] = sharedRes[offset];

            // Break if every thread is done.
            if(done)
                break;
        }
    }

    __global__ void StacklessCDSharedOther(uint                 Size,
                                           const aabb*          _box,
                                           const int*           sortedIdx,
                                           const int            intSize,
                                           const int*           _lvs_idx,
                                           const stacklessnode* _nodes,
                                           int*                 resCounter,
                                           int2*                res,
                                           const int            maxRes)
    {
        int  tid    = blockIdx.x * blockDim.x + threadIdx.x;
        bool active = tid < Size;
        int  idx;
        aabb bv;
        if(active)
        {
            idx = sortedIdx[tid];
            bv  = _box[idx];
        }

        __shared__ int2 sharedRes[MAX_RES_PER_BLOCK];
        __shared__ int sharedCounter;  // How many results are cached in shared memory
        __shared__ int sharedGlobalIdx;  // Where to write in global memory
        if(threadIdx.x == 0)
            sharedCounter = 0;

        int           st = 0;
        stacklessnode node;
        while(true)
        {
            __syncthreads();
            if(active)
            {
                while(st != -1)
                {
                    node.lc     = _nodes[st].lc;
                    node.escape = _nodes[st].escape;
                    node.bound  = _nodes[st].bound;

                    //node = _nodes[st];
                    if(node.bound.intersects(bv))
                    {
                        if(node.lc == -1)
                        {
                            int sIdx = atomicAdd(&sharedCounter, 1);
                            if(sIdx >= MAX_RES_PER_BLOCK)
                            {
                                break;
                            }

                            sharedRes[sIdx] = int2{idx, _lvs_idx[st - intSize]};
                            st              = node.escape;
                        }
                        else
                        {
                            st = node.lc;
                        }
                    }
                    else
                    {
                        st = node.escape;
                    }
                }
            }
            // Flush whatever we have
            __syncthreads();
            int totalRes = min(sharedCounter, MAX_RES_PER_BLOCK);

            if(threadIdx.x == 0)
            {
                sharedGlobalIdx = atomicAdd(resCounter, totalRes);
            }

            __syncthreads();

            // Make sure we dont write out of bounds
            const int globalIdx = sharedGlobalIdx;

            if(globalIdx >= maxRes || !totalRes)
                return;  // Out of memory for results.
            if(threadIdx.x == 0)
                sharedCounter = 0;
            //
            //// If we got here with a half full buffer, we are done.
            bool done = totalRes < MAX_RES_PER_BLOCK;
            // If we are about to run out of memory, we are done.
            if(totalRes > maxRes - globalIdx)
            {
                totalRes = maxRes - globalIdx;
                done     = true;
            }

            // Copy full blocks
            int fullBlocks = (totalRes - 1) / (int)blockDim.x;
            for(int i = 0; i < fullBlocks; i++)
            {
                int offset              = i * blockDim.x + threadIdx.x;
                res[globalIdx + offset] = sharedRes[offset];
            }

            // Copy the rest
            int offset = fullBlocks * blockDim.x + threadIdx.x;
            if(offset < totalRes)
                res[globalIdx + offset] = sharedRes[offset];

            // Break if every thread is done.
            if(done)
                break;
        }
    }
}  // namespace LBVHKernels

LBVHStackless::LBVHStackless() {}
LBVHStackless::LBVHStackless::~LBVHStackless() = default;

void LBVHStackless::compute(aabb* devicePtr, size_t size)
{
    d_objs = thrust::device_ptr<aabb>(devicePtr);
    //this will make the BVH valid once compute is called
    numObjs = size;

    const unsigned int numInternalNodes = numObjs - 1;  // Total number of internal nodes
    const unsigned int numNodes = numObjs * 2 - 1;  // Total number of nodes

    d_scene_box.resize(1);
    d_flags.resize(numInternalNodes);
    d_mtcode.resize(numObjs);
    d_sorted_id.resize(numObjs);
    d_primMap.resize(numObjs);
    d_ext_aabb.resize(numObjs);
    d_ext_idx.resize(numObjs);
    d_ext_lca.resize(numObjs + 1);
    d_ext_par.resize(numObjs);
    d_ext_mark.resize(numObjs);

    d_metric.resize(numObjs);
    d_tkMap.resize(numObjs);
    d_offsetTable.resize(numObjs);
    d_count.resize(numObjs);


    d_int_lc.resize(numInternalNodes);
    d_int_rc.resize(numInternalNodes);
    d_int_par.resize(numInternalNodes);
    d_int_range_x.resize(numInternalNodes);
    d_int_range_y.resize(numInternalNodes);
    d_int_mark.resize(numInternalNodes);
    d_int_aabb.resize(numInternalNodes);

    if(type == 0)
    {
        d_quantNode.resize(numNodes);
    }
    else
    {
        d_nodes.resize(numNodes);
    }


    // Initialize flags to 0
    thrust::fill(d_flags.begin(), d_flags.end(), 0);
    thrust::fill(thrust::device, d_ext_mark.begin(), d_ext_mark.end(), 7);
    thrust::fill(thrust::device, d_ext_lca.begin(), d_ext_lca.end(), 0);
    thrust::fill(thrust::device, d_ext_par.begin(), d_ext_par.end(), 0);


    int blockDim = K_THREADS;
    int gridDim  = (numObjs + blockDim - 1) / blockDim;

    LBVHKernels::calcMaxBVFromBox<<<dim3(gridDim, 1, 1), dim3(blockDim, 1, 1)>>>(
        numObjs, devicePtr, thrust::raw_pointer_cast(d_scene_box.data()));

    LBVHKernels::calcMCsFromBox<<<dim3(gridDim, 1, 1), dim3(blockDim, 1, 1)>>>(
        numObjs,
        devicePtr,
        thrust::raw_pointer_cast(d_scene_box.data()),
        thrust::raw_pointer_cast(d_mtcode.data()));

    thrust::sequence(thrust::device, d_sorted_id.begin(), d_sorted_id.end());
    thrust::sort_by_key(
        thrust::device, d_mtcode.begin(), d_mtcode.end(), d_sorted_id.begin());


    LBVHKernels::calcInverseMapping<<<dim3(gridDim, 1, 1), dim3(blockDim, 1, 1)>>>(
        numObjs,
        thrust::raw_pointer_cast(d_sorted_id.data()),
        thrust::raw_pointer_cast(d_primMap.data()));

    LBVHKernels::buildPrimitivesFromBox<<<gridDim, blockDim>>>(
        numObjs,
        thrust::raw_pointer_cast(d_ext_idx.data()),
        thrust::raw_pointer_cast(d_ext_aabb.data()),
        thrust::raw_pointer_cast(d_primMap.data()),
        devicePtr);

    LBVHKernels::calcExtNodeSplitMetrics<<<gridDim, blockDim>>>(
        numObjs,
        thrust::raw_pointer_cast(d_mtcode.data()),
        thrust::raw_pointer_cast(d_metric.data()));

    // Build internal nodes
    LBVHKernels::buildIntNodes<<<(numObjs + 255) / 256, 256>>>(
        numObjs,
        thrust::raw_pointer_cast(d_count.data()),
        thrust::raw_pointer_cast(d_ext_lca.data()),
        thrust::raw_pointer_cast(d_metric.data()),
        thrust::raw_pointer_cast(d_ext_par.data()),
        thrust::raw_pointer_cast(d_ext_mark.data()),
        thrust::raw_pointer_cast(d_ext_aabb.data()),
        thrust::raw_pointer_cast(d_int_rc.data()),
        thrust::raw_pointer_cast(d_int_lc.data()),
        thrust::raw_pointer_cast(d_int_range_y.data()),
        thrust::raw_pointer_cast(d_int_range_x.data()),
        thrust::raw_pointer_cast(d_int_mark.data()),
        thrust::raw_pointer_cast(d_int_aabb.data()),
        thrust::raw_pointer_cast(d_flags.data()),
        thrust::raw_pointer_cast(d_int_par.data()));

    thrust::exclusive_scan(
        thrust::device, d_count.begin(), d_count.end(), d_offsetTable.begin());


    // Compute node orders
    LBVHKernels::calcIntNodeOrders<<<gridDim, blockDim>>>(
        numObjs,
        thrust::raw_pointer_cast(d_int_lc.data()),
        thrust::raw_pointer_cast(d_ext_lca.data()),
        thrust::raw_pointer_cast(d_count.data()),
        thrust::raw_pointer_cast(d_offsetTable.data()),
        thrust::raw_pointer_cast(d_tkMap.data()));


    thrust::fill(thrust::device, d_ext_lca.begin() + numObjs, d_ext_lca.begin() + numObjs + 1, -1);
    //LBVHKernels::setEscape << <1, 1 >> > (thrust::raw_pointer_cast(d_ext_lca.data()), numObjs);

    // Update external node links
    LBVHKernels::updateBvhExtNodeLinks<<<gridDim, blockDim>>>(
        numObjs,
        thrust::raw_pointer_cast(d_tkMap.data()),
        thrust::raw_pointer_cast(d_ext_lca.data()),
        thrust::raw_pointer_cast(d_ext_par.data()));

    if(type == 0)
    {
        LBVHKernels::reorderQuantilizedNode<<<gridDim, blockDim>>>(
            numInternalNodes,
            thrust::raw_pointer_cast(d_tkMap.data()),
            thrust::raw_pointer_cast(d_ext_lca.data()),
            thrust::raw_pointer_cast(d_ext_aabb.data()),
            thrust::raw_pointer_cast(d_int_lc.data()),
            thrust::raw_pointer_cast(d_int_mark.data()),
            thrust::raw_pointer_cast(d_int_range_y.data()),
            thrust::raw_pointer_cast(d_int_aabb.data()),
            thrust::raw_pointer_cast(d_scene_box.data()),
            thrust::raw_pointer_cast(d_quantNode.data()));
    }
    else
    {
        LBVHKernels::reorderNode<<<gridDim, blockDim>>>(
            numInternalNodes,
            thrust::raw_pointer_cast(d_tkMap.data()),
            thrust::raw_pointer_cast(d_ext_lca.data()),
            thrust::raw_pointer_cast(d_ext_aabb.data()),
            thrust::raw_pointer_cast(d_int_lc.data()),
            thrust::raw_pointer_cast(d_int_mark.data()),
            thrust::raw_pointer_cast(d_int_range_y.data()),
            thrust::raw_pointer_cast(d_int_aabb.data()),
            thrust::raw_pointer_cast(d_nodes.data()));
    }
}

size_t LBVHStackless::query()
{
    // Borrow the flags array for the counter
    if(!allocated)
    {
        cudaMalloc((void**)&d_cpNum, sizeof(int));
        cudaMalloc((void**)&d_cpRes, sizeof(int2) * max_cpNum);
        allocated = true;
    }
    checkCudaErrors(cudaGetLastError());
    // Query the LBVH
    const int numQuery = numObjs;

    if(type == 0)
    {
        LBVHKernels::quantilizedStacklessCDSharedSelf<<<(numQuery + 255) / 256, 256>>>(
            numQuery,
            thrust::raw_pointer_cast(d_objs),
            numObjs - 1,
            thrust::raw_pointer_cast(d_ext_idx.data()),
            thrust::raw_pointer_cast(d_scene_box.data()),
            thrust::raw_pointer_cast(d_quantNode.data()),
            d_cpNum,
            d_cpRes,
            max_cpNum);
    }
    else
    {
        LBVHKernels::StacklessCDSharedSelf<<<(numQuery + 255) / 256, 256>>>(
            numQuery,
            thrust::raw_pointer_cast(d_objs),
            numObjs - 1,
            thrust::raw_pointer_cast(d_ext_idx.data()),
            thrust::raw_pointer_cast(d_nodes.data()),
            d_cpNum,
            d_cpRes,
            max_cpNum);
    }


    cudaMemcpy((void*)&h_cpNum, d_cpNum, sizeof(int), cudaMemcpyDeviceToHost);
    checkCudaErrors(cudaGetLastError());

    printf("Total number of collisions: %d\n", h_cpNum);

    return h_cpNum;
}

size_t LBVHStackless::queryOther(aabb* devicePtr, size_t size)
{
    // Borrow the flags array for the counter
    if(!allocated)
    {
        cudaMalloc((void**)&d_cpNum, sizeof(int));
        cudaMalloc((void**)&d_cpRes, sizeof(int2) * max_cpNum);
        allocated = true;
    }
    if(queryNum == 0)
    {
        cudaMalloc((void**)&d_querySceneBox, sizeof(aabb));
        cudaMalloc((void**)&d_queryMtCode, sizeof(uint32_t) * size);
        cudaMalloc((void**)&d_querySortedId, sizeof(int) * size);
    }
    else if(queryNum < size)
    {
        cudaFree(d_querySceneBox);
        cudaFree(d_queryMtCode);
        cudaFree(d_querySortedId);
        cudaMalloc((void**)&d_querySceneBox, sizeof(aabb));
        cudaMalloc((void**)&d_queryMtCode, sizeof(uint32_t) * size);
        cudaMalloc((void**)&d_querySortedId, sizeof(int) * size);
        queryNum = size;
    }

    checkCudaErrors(cudaGetLastError());
    // Query the LBVH
    const int numQuery = size;

    LBVHKernels::calcMaxBVFromBox<<<(numQuery + 255) / 256, 256>>>(numQuery, devicePtr, d_querySceneBox);
    checkCudaErrors(cudaDeviceSynchronize());

    LBVHKernels::calcMCsFromBox<<<(numQuery + 255) / 256, 256>>>(
        numQuery, devicePtr, d_querySceneBox, d_queryMtCode);
    checkCudaErrors(cudaDeviceSynchronize());

    thrust::sequence(thrust::device, d_querySortedId, d_querySortedId + numQuery);
    checkCudaErrors(cudaDeviceSynchronize());

    thrust::sort_by_key(thrust::device, d_queryMtCode, d_queryMtCode + numQuery, d_querySortedId);
    checkCudaErrors(cudaDeviceSynchronize());

    if(type == 0)
    {
        LBVHKernels::quantilizedStacklessCDSharedOther<<<(numQuery + 255) / 256, 256>>>(
            numQuery,
            devicePtr,
            d_querySortedId,
            numObjs - 1,
            thrust::raw_pointer_cast(d_ext_idx.data()),
            thrust::raw_pointer_cast(d_scene_box.data()),
            thrust::raw_pointer_cast(d_quantNode.data()),
            d_cpNum,
            d_cpRes,
            max_cpNum);
        checkCudaErrors(cudaDeviceSynchronize());
    }
    else
    {
        LBVHKernels::StacklessCDSharedOther<<<(numQuery + 255) / 256, 256>>>(
            numQuery,
            devicePtr,
            d_querySortedId,
            numObjs - 1,
            thrust::raw_pointer_cast(d_ext_idx.data()),
            thrust::raw_pointer_cast(d_nodes.data()),
            d_cpNum,
            d_cpRes,
            max_cpNum);
        checkCudaErrors(cudaDeviceSynchronize());
    }
    cudaMemcpy((void*)&h_cpNum, d_cpNum, sizeof(int), cudaMemcpyDeviceToHost);
    checkCudaErrors(cudaDeviceSynchronize());

    printf("Total number of collisions: %d\n", h_cpNum);

    return h_cpNum;
}

std::vector<int2> brute_froce_cp(const std::vector<culbvh::Bound<float>>& aabbs)
{
    std::vector<int2> pairs;
    for(int i = 0; i < aabbs.size(); ++i)
    {
        auto aabb0 = aabbs[i];
        for(int j = 0; j < aabbs.size(); ++j)
        {
            auto aabb1 = aabbs[j];
            if(aabb1.intersects(aabb0))
            {
                pairs.push_back(make_int2(i, j));
            }
        }
    }
    return pairs;
}

void testStacklessLBVH()
{
    const int   N = 50000;
    const float R = 0.001f;

    printf("Generating Data...\n");
    vector<culbvh::Bound<float>> points(N);

    srand(1);
    for(size_t i = 0; i < N; i++)
    {
        culbvh::Bound<float> b(make_float3(rand() / (float)RAND_MAX,
                                           rand() / (float)RAND_MAX,
                                           rand() / (float)RAND_MAX));
        //culbvh::Bound<float> b(make_float3(i * R * 1.1f, i * R * 1.1f, i * R * 1.1f));
        b.pad(R);
        points[i] = b;
    }

    // Brute-force
    printf("Brute-force collision detection...\n");

    auto cps = brute_froce_cp(points);
    printf("Total number of collisions (brute-force): %zu\n", cps.size());


    thrust::device_vector<culbvh::Bound<float>> d_points(points.begin(), points.end());
    thrust::device_vector<int2> d_res(100 * N);
    cudaDeviceSynchronize();

    // Create CUDA events for timing
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    float milliseconds = 0;

    // Build BVH
    printf("Building LBVH...\n");
    cudaEventRecord(start);
    LBVHStackless bvh;
    bvh.type = 1;
    bvh.compute(thrust::raw_pointer_cast(d_points.data()), N);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("LBVH build time: %.3f ms\n", milliseconds);
    // Query BVH
    printf("Querying LBVH...\n");
    cudaEventRecord(start);
    //int numCols = bvh.query();
    int numCols = bvh.queryOther(thrust::raw_pointer_cast(d_points.data()), N);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("LBVH query time: %.3f ms\n", milliseconds);
}
}  // namespace culbvh