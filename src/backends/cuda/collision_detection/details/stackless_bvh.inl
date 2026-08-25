#include <cuda_device/builtin.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::culbvh
{
using aabb          = uipc::backend::cuda::AABB;
using stacklessnode = uipc::backend::cuda::StacklessBVH::Node;
using Vector2i      = uipc::Vector2i;

using uint   = uint32_t;
using ullint = unsigned long long int;

constexpr int K_THREADS = 256;
constexpr int K_WARPS   = K_THREADS >> 5;

constexpr int K_REDUCTION_LAYER  = 5;
constexpr int K_REDUCTION_NUM    = 1 << K_REDUCTION_LAYER;
constexpr int K_REDUCTION_MODULO = K_REDUCTION_NUM - 1;

constexpr int    aabbBits  = 15;
constexpr int    aabbRes   = (1 << aabbBits) - 2;
constexpr int    indexBits = 64 - 3 * aabbBits;
constexpr int    offset3   = aabbBits * 3;
constexpr int    offset2   = aabbBits * 2;
constexpr int    offset1   = aabbBits * 1;
constexpr ullint indexMask = 0xFFFFFFFFFFFFFFFFu << offset3;
constexpr uint   aabbMask  = 0xFFFFFFFFu >> (32 - aabbBits);
constexpr uint   MaxIndex  = 0xFFFFFFFFFFFFFFFFu >> offset3;

constexpr uint MAX_CD_NUM_PER_VERT = 64;
constexpr int  MAX_RES_PER_BLOCK   = 1024;

struct PlainAABB
{
    float3 _min, _max;
};

UIPC_GENERIC UIPC_INLINE PlainAABB toPlainAABB(const aabb& box)
{
    PlainAABB res;
    res._min = make_float3(box.min().x(), box.min().y(), box.min().z());
    res._max = make_float3(box.max().x(), box.max().y(), box.max().z());
    return res;
}

UIPC_GENERIC UIPC_INLINE aabb fromPlainAABB(const PlainAABB& box)
{
    aabb aabb;
    aabb.min() = Vector<float, 3>(box._min.x, box._min.y, box._min.z);
    aabb.max() = Vector<float, 3>(box._max.x, box._max.y, box._max.z);
    return aabb;
}

struct intAABB
{
    int3 _min, _max;

    UIPC_GENERIC UIPC_INLINE void convertFrom(const aabb& other, float3& origin, float3& delta)
    {
        _min.x = static_cast<int>((other.min().x() - origin.x) / delta.x);
        _min.y = static_cast<int>((other.min().y() - origin.y) / delta.y);
        _min.z = static_cast<int>((other.min().z() - origin.z) / delta.z);
        _max.x = static_cast<int>(ceilf((other.max().x() - origin.x) / delta.x));
        _max.y = static_cast<int>(ceilf((other.max().y() - origin.y) / delta.y));
        _max.z = static_cast<int>(ceilf((other.max().z() - origin.z) / delta.z));
    }
};

template <typename T>
UIPC_GENERIC UIPC_INLINE T __mm_min(T a, T b)
{
    return a > b ? b : a;
}

template <typename T>
UIPC_GENERIC UIPC_INLINE T __mm_max(T a, T b)
{
    return a > b ? a : b;
}

UIPC_DEVICE UIPC_INLINE float atomicMinf(float* addr, float value)
{
    float old;
    old = (value >= 0) ?
              __int_as_float(atomicMin((int*)addr, __float_as_int(value))) :
              __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));
    return old;
}

UIPC_DEVICE UIPC_INLINE float atomicMaxf(float* addr, float value)
{
    float old;
    old = (value >= 0) ?
              __int_as_float(atomicMax((int*)addr, __float_as_int(value))) :
              __uint_as_float(atomicMin((unsigned int*)addr, __float_as_uint(value)));
    return old;
}

UIPC_GENERIC UIPC_INLINE uint expandBits(uint v)
{  ///< Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

UIPC_GENERIC UIPC_INLINE uint morton3D(float x, float y, float z)
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
UIPC_GENERIC UIPC_INLINE bool lessThan(const int3& a, const int3& b)
{
    if(a.x != b.x)
        return a.x < b.x;
    if(a.y != b.y)
        return a.y < b.y;
    return a.z < b.z;
}

UIPC_GENERIC UIPC_INLINE Vector2i to_eigen(int2 v)
{
    return Vector2i{v.x, v.y};
}

UIPC_GENERIC UIPC_INLINE int2 make_ordered_pair(int a, int b)
{
    if(a < b)
        return int2{a, b};
    else
        return int2{b, a};
}

UIPC_GENERIC UIPC_INLINE float3 operator-(const float3& v0, const float3& v1)
{
    return make_float3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

UIPC_GENERIC UIPC_INLINE void SafeCopyTo(int2* sharedRes,
                                         int   totalResInBlock,

                                         Vector2i* globalRes,
                                         int       globalIdx,
                                         int       maxRes)
{
    if(globalIdx >= maxRes      // Out of memory for results.
       || totalResInBlock == 0  // No results to write
    )
        return;

    auto CopyCount = std::min(totalResInBlock, maxRes - globalIdx);

    // Copy full blocks
    int fullBlocks = (CopyCount - 1) / (int)blockDim.x;
    for(int i = 0; i < fullBlocks; i++)
    {
        int offset                    = i * blockDim.x + threadIdx.x;
        globalRes[globalIdx + offset] = to_eigen(sharedRes[offset]);
    }

    // Copy the rest
    int offset = fullBlocks * blockDim.x + threadIdx.x;
    if(offset < CopyCount)
        globalRes[globalIdx + offset] = to_eigen(sharedRes[offset]);
}

}  // namespace uipc::culbvh

namespace uipc::backend::cuda
{
namespace
{
    __global__ void StacklessBVH_initializeBuildState_kernel(
        int                             num_objs,
        cuda_tool::Dense<AABB>          scene_box,
        cuda_tool::BufferView<uint32_t> flags,
        cuda_tool::BufferView<uint32_t> ext_mark,
        cuda_tool::BufferView<int>      ext_lca,
        cuda_tool::BufferView<uint32_t> ext_par,
        cuda_tool::BufferView<uint32_t> depths,
        cuda_tool::BufferView<int32_t>  unsorted_ids)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx == 0)
            *scene_box = AABB();
        if(idx < num_objs - 1)
            flags(idx) = 0;
        if(idx < num_objs)
        {
            ext_mark(idx)     = 7;
            ext_lca(idx)      = 0;
            ext_par(idx)      = 0;
            depths(idx)       = 0;
            unsorted_ids(idx) = idx;
        }
        if(idx == num_objs)
            ext_lca(idx) = -1;
    }

    __global__ void StacklessBVH_initializeQueryState_kernel(int num_objs,
                                                             cuda_tool::Dense<AABB> scene_box,
                                                             cuda_tool::BufferView<int> unsorted_ids)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx == 0)
            *scene_box = AABB();
        if(idx < num_objs)
            unsorted_ids(idx) = idx;
    }

    __global__ void StacklessBVH_calcMaxBVFromBox_kernel(size_t size,
                                                         cuda_tool::CBufferView<AABB> box,
                                                         cuda_tool::Dense<AABB> _bv)
    {
        using namespace culbvh;
        int idx     = blockIdx.x * blockDim.x + threadIdx.x;
        int warpTid = threadIdx.x % 32;
        int warpId  = (threadIdx.x >> 5);
        int warpNum;
        if(idx >= size)
            return;
        __shared__ PlainAABB aabbData[K_WARPS];

        PlainAABB temp = toPlainAABB(box(idx));
        __syncthreads();

        // Extract values for warp shuffle
        float tempMinX = temp._min.x;
        float tempMinY = temp._min.y;
        float tempMinZ = temp._min.z;
        float tempMaxX = temp._max.x;
        float tempMaxY = temp._max.y;
        float tempMaxZ = temp._max.z;

        for(int i = 1; i < 32; i = (i << 1))
        {
            float otherMinX = __shfl_down_sync(0xffffffff, tempMinX, i);
            float otherMinY = __shfl_down_sync(0xffffffff, tempMinY, i);
            float otherMinZ = __shfl_down_sync(0xffffffff, tempMinZ, i);
            float otherMaxX = __shfl_down_sync(0xffffffff, tempMaxX, i);
            float otherMaxY = __shfl_down_sync(0xffffffff, tempMaxY, i);
            float otherMaxZ = __shfl_down_sync(0xffffffff, tempMaxZ, i);
            tempMinX        = __mm_min(tempMinX, otherMinX);
            tempMinY        = __mm_min(tempMinY, otherMinY);
            tempMinZ        = __mm_min(tempMinZ, otherMinZ);
            tempMaxX        = __mm_max(tempMaxX, otherMaxX);
            tempMaxY        = __mm_max(tempMaxY, otherMaxY);
            tempMaxZ        = __mm_max(tempMaxZ, otherMaxZ);
        }

        if(blockIdx.x == gridDim.x - 1)
        {
            warpNum = ((size - blockIdx.x * blockDim.x + 31) >> 5);
        }
        else
        {
            warpNum = ((blockDim.x) >> 5);
        }

        if(warpTid == 0)
        {
            // Reconstruct AABB from reduced values
            aabbData[warpId]._min = make_float3(tempMinX, tempMinY, tempMinZ);
            aabbData[warpId]._max = make_float3(tempMaxX, tempMaxY, tempMaxZ);
        }
        __syncthreads();
        if(threadIdx.x >= warpNum)
            return;

        if(warpNum > 1)
        {
            temp     = aabbData[threadIdx.x];
            tempMinX = temp._min.x;
            tempMinY = temp._min.y;
            tempMinZ = temp._min.z;
            tempMaxX = temp._max.x;
            tempMaxY = temp._max.y;
            tempMaxZ = temp._max.z;

            for(int i = 1; i < warpNum; i = (i << 1))
            {
                float otherMinX = __shfl_down_sync(0xffffffff, tempMinX, i);
                float otherMinY = __shfl_down_sync(0xffffffff, tempMinY, i);
                float otherMinZ = __shfl_down_sync(0xffffffff, tempMinZ, i);
                float otherMaxX = __shfl_down_sync(0xffffffff, tempMaxX, i);
                float otherMaxY = __shfl_down_sync(0xffffffff, tempMaxY, i);
                float otherMaxZ = __shfl_down_sync(0xffffffff, tempMaxZ, i);
                tempMinX        = __mm_min(tempMinX, otherMinX);
                tempMinY        = __mm_min(tempMinY, otherMinY);
                tempMinZ        = __mm_min(tempMinZ, otherMinZ);
                tempMaxX        = __mm_max(tempMaxX, otherMaxX);
                tempMaxY        = __mm_max(tempMaxY, otherMaxY);
                tempMaxZ        = __mm_max(tempMaxZ, otherMaxZ);
            }
        }

        if(threadIdx.x == 0)
        {
            atomicMinf(&_bv->min().x(), tempMinX);
            atomicMinf(&_bv->min().y(), tempMinY);
            atomicMinf(&_bv->min().z(), tempMinZ);
            atomicMaxf(&_bv->max().x(), tempMaxX);
            atomicMaxf(&_bv->max().y(), tempMaxY);
            atomicMaxf(&_bv->max().z(), tempMaxZ);
        }
    }

    __global__ void StacklessBVH_calcMCsFromBox_kernel(cuda_tool::CBufferView<AABB> box,
                                                       cuda_tool::CDense<AABB> scene,
                                                       cuda_tool::BufferView<uint32_t> codes,
                                                       int n)
    {
        using namespace culbvh;
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        AABB bv = box(idx);

        // Get center using Eigen API
        auto   center = bv.center();
        float3 c      = make_float3(center.x(), center.y(), center.z());

        // Get scene min
        auto sceneMin = scene->min();
        float3 sceneMinVec = make_float3(sceneMin.x(), sceneMin.y(), sceneMin.z());
        const float3 offset = c - sceneMinVec;

        // Get dimensions
        auto sceneSize = scene->sizes();
        codes(idx)     = morton3D(offset.x / sceneSize.x(),
                              offset.y / sceneSize.y(),
                              offset.z / sceneSize.z());
    }

    /// incoherent access, thus poor performance
    __global__ void StacklessBVH_calcInverseMapping_kernel(
        cuda_tool::BufferView<int32_t> map, cuda_tool::BufferView<int32_t> invMap, int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        //
        invMap(map(idx)) = idx;
    }

    __global__ void StacklessBVH_buildPrimitivesFromBox_kernel(
        cuda_tool::BufferView<int>     _primIdx,
        cuda_tool::BufferView<AABB>    _primBox,
        cuda_tool::BufferView<int32_t> _primMap,
        cuda_tool::CBufferView<AABB>   box,
        int                            n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        int  newIdx      = _primMap(idx);
        AABB bv          = box(idx);
        _primIdx(newIdx) = idx;
        _primBox(newIdx) = bv;
    }

    __global__ void StacklessBVH_calcExtNodeSplitMetrics_kernel(size_t extsize,
                                                                cuda_tool::BufferView<uint32_t> _codes,
                                                                cuda_tool::BufferView<int> _metrics,
                                                                int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        _metrics(idx) =
            idx != extsize - 1 ? 32 - __clz(_codes(idx) ^ _codes(idx + 1)) : 33;
    }

    __global__ void StacklessBVH_buildIntNodes_kernel(
        int size,
        // leaf nodes
        cuda_tool::BufferView<uint32_t> _depths,
        cuda_tool::BufferView<int>      _lvs_lca,
        cuda_tool::BufferView<int>      _lvs_metric,
        cuda_tool::BufferView<uint32_t> _lvs_par,
        cuda_tool::BufferView<uint32_t> _lvs_mark,
        cuda_tool::BufferView<AABB>     _lvs_box,
        // internal nodes
        cuda_tool::BufferView<int>      _tks_rc,
        cuda_tool::BufferView<int>      _tks_lc,
        cuda_tool::BufferView<int>      _tks_range_y,
        cuda_tool::BufferView<int>      _tks_range_x,
        cuda_tool::BufferView<uint32_t> _tks_mark,
        cuda_tool::BufferView<AABB>     _tks_box,
        cuda_tool::BufferView<uint32_t> _flag,
        cuda_tool::BufferView<int>      _tks_par)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;

        _lvs_lca(idx) = -1, _depths(idx) = 0;
        int  l = idx - 1, r = idx;  ///< (l, r]
        bool mark;
        if(l >= 0)
            mark = _lvs_metric(l) < _lvs_metric(r);  //determine direction
        else
            mark = false;
        int cur = mark ? l : r;

        _lvs_par(idx) = cur;


        if(_flag.total_size() == 0)
            // when we only have 1 external node
            // there is no internal node to build
            return;

        if(mark)
        {
            _tks_rc(cur)      = idx;
            _tks_range_y(cur) = idx;
            atomicOr(&_tks_mark(cur), 0x00000002);
            _lvs_mark(idx) = 0x00000007;
        }
        else
        {
            _tks_lc(cur)      = idx;
            _tks_range_x(cur) = idx;
            atomicOr(&_tks_mark(cur), 0x00000001);
            _lvs_mark(idx) = 0x00000003;
        }

        __threadfence();
        while(atomicAdd(&_flag(cur), 1) == 1)
        {
            //_tks.update(cur, _lvs);	/// Update
            //_tks.refit(cur, _lvs);	/// Refit
            int      chl       = _tks_lc(cur);
            int      chr       = _tks_rc(cur);
            uint32_t temp_mark = _tks_mark(cur);
            if(temp_mark & 1)
            {
                _tks_box(cur) = _lvs_box(chl);
            }
            else
            {
                _tks_box(cur) = _tks_box(chl);
            }
            if(temp_mark & 2)
            {
                _tks_box(cur).extend(_lvs_box(chr));
            }
            else
            {
                _tks_box(cur).extend(_tks_box(chr));
            }

            _tks_mark(cur) &= 0x00000007;

            l               = _tks_range_x(cur) - 1;
            r               = _tks_range_y(cur);
            _lvs_lca(l + 1) = cur;
            _depths(l + 1)++;
            if(l >= 0)
            {
                mark = _lvs_metric(l) < _lvs_metric(r);  ///< true when right child, false otherwise
            }
            else
            {
                mark = false;
            }

            if(l + 1 == 0 && r == size - 1)
            {
                _tks_par(cur) = -1;
                _tks_mark(cur) &= 0xFFFFFFFB;
                break;
            }

            int par       = mark ? l : r;
            _tks_par(cur) = par;
            if(mark)
            {
                _tks_rc(par)      = cur;
                _tks_range_y(par) = r;
                atomicAnd(&_tks_mark(par), 0xFFFFFFFD);
                _tks_mark(cur) |= 0x00000004;
            }
            else
            {
                _tks_lc(par)      = cur;
                _tks_range_x(par) = l + 1;
                atomicAnd(&_tks_mark(par), 0xFFFFFFFE);
                _tks_mark(cur) &= 0xFFFFFFFB;
            }
            __threadfence();
            cur = par;
        }
    }

    __global__ void StacklessBVH_calcIntNodeOrders_kernel(
        cuda_tool::BufferView<int>      _tks_lc,
        cuda_tool::BufferView<int>      _lcas,
        cuda_tool::BufferView<uint32_t> _depths,
        cuda_tool::BufferView<uint32_t> _offsets,
        cuda_tool::BufferView<int>      _tkMap,
        int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        int node  = _lcas(idx);
        int depth = _depths(idx);
        int id    = _offsets(idx);

        if(node != -1)
        {
            for(; depth--; node = _tks_lc(node))
            {
                _tkMap(node) = id++;
            }
        }
    }

    __global__ void StacklessBVH_updateBvhExtNodeLinks_kernel(
        cuda_tool::BufferView<int>      _mapTable,
        cuda_tool::BufferView<int>      _lcas,
        cuda_tool::BufferView<uint32_t> _pars,
        int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        int ori;
        _pars(idx) = _mapTable(_pars(idx));
        if((ori = _lcas(idx)) != -1)
            _lcas(idx) = _mapTable(ori) << 1;
        else
            _lcas(idx) = idx << 1 | 1;
    }

    __global__ void StacklessBVH_reorderNode_kernel(
        int intSize,
        // leaf nodes
        cuda_tool::BufferView<int>  _lvs_lca,
        cuda_tool::BufferView<AABB> _lvs_box,
        // internal nodes
        cuda_tool::BufferView<int>      _tkMap,
        cuda_tool::BufferView<int>      _unorderedTks_lc,
        cuda_tool::BufferView<uint32_t> _unorderedTks_mark,
        cuda_tool::BufferView<int>      _unorderedTks_rangey,
        cuda_tool::BufferView<AABB>     _unorderedTks_box,
        // total nodes
        cuda_tool::BufferView<StacklessBVH::Node> _nodes,
        int                                       n)
    {
        using namespace culbvh;
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        stacklessnode Node;
        Node.lc    = -1;
        int escape = _lvs_lca(idx + 1);

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
        Node.bound = _lvs_box(idx);


        _nodes(idx + intSize) = Node;

        if(idx >= intSize)
            return;

        stacklessnode internalNode;
        int           newId = _tkMap(idx);
        uint32_t      mark  = _unorderedTks_mark(idx);

        internalNode.lc    = mark & 1 ? _unorderedTks_lc(idx) + intSize :
                                        _tkMap(_unorderedTks_lc(idx));
        internalNode.bound = _unorderedTks_box(idx);

        int internalEscape = _lvs_lca(_unorderedTks_rangey(idx) + 1);

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
        _nodes(newId) = internalNode;
    }

    template <typename Pred>
    __global__ void StacklessBVH_StacklessCDSharedSelf_kernel(
        int                                       Size,
        cuda_tool::CBufferView<AABB>              _box,
        int                                       intSize,
        int                                       numObjs,
        cuda_tool::BufferView<int>                _lvs_idx,
        cuda_tool::BufferView<StacklessBVH::Node> _nodes,
        cuda_tool::Dense<int>                     resCounter,
        cuda_tool::BufferView<Vector2i>           res,
        Pred                                      pred)
    {
        using namespace culbvh;
        int  tid    = blockIdx.x * blockDim.x + threadIdx.x;
        bool active = tid < Size;
        int  idx;
        AABB bv;
        if(active)
        {
            idx = _lvs_idx(tid);
            bv  = _box(idx);
        }

        __shared__ int2 sharedRes[MAX_RES_PER_BLOCK];
        __shared__ int sharedCounter;  // How many results are cached in shared memory
        __shared__ int sharedGlobalIdx;  // Where to write in global memory
        if(threadIdx.x == 0)
            sharedCounter = 0;

        int                st = 0;
        StacklessBVH::Node node;
        // Upper bound of iterations to avoid infinite loop
        const int MaxIter = numObjs * 2;

        while(true)
        {
            __syncthreads();
            if(active)
            {
                int inner_I = 0;
                for(; inner_I < MaxIter; inner_I++)
                {
                    if(st == -1)
                        break;
                    // Load node data - Eigen::AlignedBox stores min and max as Vector3f members
                    node.lc     = _nodes(st).lc;
                    node.escape = _nodes(st).escape;
                    node.bound  = _nodes(st).bound;
                    //node = _nodes[st];
                    if(node.bound.intersects(bv))
                    {
                        if(node.lc == -1)
                        {
                            if(tid < st - intSize)
                            {
                                auto pair = make_ordered_pair(idx, _lvs_idx(st - intSize));
                                if(pred(pair.x, pair.y))
                                {
                                    int sIdx = atomicAdd(&sharedCounter, 1);
                                    if(sIdx >= MAX_RES_PER_BLOCK)
                                    {
                                        break;
                                    }

                                    sharedRes[sIdx] = pair;
                                }
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

                UIPC_KERNEL_ASSERT(inner_I < MaxIter,
                                   "Exceeded max iteration in stackless traversal, %d (Max=%d), numObj=(%d)",
                                   inner_I,
                                   MaxIter,
                                   numObjs);
            }
            // Flush whatever we have
            __syncthreads();

            int totalResInBlock = min(sharedCounter, MAX_RES_PER_BLOCK);

            if(threadIdx.x == 0)
            {
                // This Block Starts writing at sharedGlobalIdx
                sharedGlobalIdx = atomicAdd(resCounter.data(), totalResInBlock);
            }

            __syncthreads();

            // Make sure we dont write out of bounds
            const int globalIdx = sharedGlobalIdx;

            if(threadIdx.x == 0)
                sharedCounter = 0;

            // if there is at least one element empty
            // it means we have found all collisions for this block
            bool done = totalResInBlock < MAX_RES_PER_BLOCK;

            SafeCopyTo(sharedRes,
                       totalResInBlock,
                       res.data(),
                       globalIdx,
                       static_cast<int>(res.total_size()));

            if(done)
                break;
        }
    }

    template <typename Pred>
    __global__ void StacklessBVH_StacklessCDSharedOther_kernel(
        int                                       Size,
        cuda_tool::CBufferView<AABB>              _box,
        cuda_tool::CBufferView<int>               sortedIdx,
        int                                       intSize,
        int                                       numObjs,
        cuda_tool::BufferView<int>                _lvs_idx,
        cuda_tool::BufferView<StacklessBVH::Node> _nodes,
        cuda_tool::Dense<int>                     resCounter,
        cuda_tool::BufferView<Vector2i>           res,
        Pred                                      pred)
    {
        using namespace culbvh;
        int  tid    = blockIdx.x * blockDim.x + threadIdx.x;
        bool active = tid < Size;
        int  idx;
        AABB bv;
        if(active)
        {
            idx = sortedIdx(tid);
            bv  = _box(idx);
        }

        __shared__ int2 sharedRes[MAX_RES_PER_BLOCK];
        __shared__ int sharedCounter;  // How many results are cached in shared memory
        __shared__ int sharedGlobalIdx;  // Where to write in global memory
        if(threadIdx.x == 0)
            sharedCounter = 0;

        int                st = 0;
        StacklessBVH::Node node;

        // Upper bound of iterations to avoid infinite loop
        const int MaxIter = numObjs * 2;

        while(true)
        {
            __syncthreads();
            if(active)
            {
                int inner_I = 0;
                for(; inner_I < MaxIter; inner_I++)
                {
                    if(st == -1)
                        break;

                    node.lc     = _nodes(st).lc;
                    node.escape = _nodes(st).escape;
                    node.bound  = _nodes(st).bound;

                    //node = _nodes[st];
                    if(node.bound.intersects(bv))
                    {
                        if(node.lc == -1)
                        {
                            auto pair = int2{idx, _lvs_idx(st - intSize)};
                            if(pred(pair.x, pair.y))
                            {
                                int sIdx = atomicAdd(&sharedCounter, 1);

                                if(sIdx >= MAX_RES_PER_BLOCK)
                                {
                                    break;
                                }

                                sharedRes[sIdx] = pair;
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

                UIPC_KERNEL_ASSERT(inner_I < MaxIter,
                                   "Exceeded max iteration in stackless traversal, %d (Max=%d), numObj=(%d)",
                                   inner_I,
                                   MaxIter,
                                   numObjs);
            }


            // Flush whatever we have
            __syncthreads();
            int totalResInBlock = min(sharedCounter, MAX_RES_PER_BLOCK);

            if(threadIdx.x == 0)
            {
                // This Block Starts writing at sharedGlobalIdx
                sharedGlobalIdx = atomicAdd(resCounter.data(), totalResInBlock);
            }

            __syncthreads();

            // Make sure we dont write out of bounds
            const int globalIdx = sharedGlobalIdx;

            if(threadIdx.x == 0)
                sharedCounter = 0;

            __syncthreads();

            // if there is at least one element empty
            // it means we have found all collisions for this block
            bool done = totalResInBlock < MAX_RES_PER_BLOCK;

            SafeCopyTo(sharedRes,
                       totalResInBlock,
                       res.data(),
                       globalIdx,
                       static_cast<int>(res.total_size()));

            if(done)
                break;
        }
    }
}  // namespace

UIPC_INLINE void StacklessBVH::Impl::calcMaxBVFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                                      cuda_tool::VarView<AABB> scene_box)
{
    using namespace culbvh;

    auto numQuery = aabbs.size();
    auto BlockDim = K_THREADS;
    auto GridDim  = (numQuery + BlockDim - 1) / BlockDim;

    if(GridDim > 0)
        StacklessBVH_calcMaxBVFromBox_kernel<<<GridDim, BlockDim, 0, nullptr>>>(
            aabbs.size(), aabbs, scene_box.viewer());
}

UIPC_INLINE void StacklessBVH::Impl::calcMCsFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                                    cuda_tool::CVarView<AABB> scene_box,
                                                    cuda_tool::BufferView<uint32_t> codes)
{
    auto k = StacklessBVH_calcMCsFromBox_kernel;
    int  n = static_cast<int>(aabbs.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            aabbs, scene_box.viewer(), codes, n);
}

/// incoherent access, thus poor performance
UIPC_INLINE void StacklessBVH::Impl::calcInverseMapping()
{
    auto k = StacklessBVH_calcInverseMapping_kernel;
    int  n = static_cast<int>(sorted_id.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            sorted_id.view(), primMap.view(), n);
}

UIPC_INLINE void StacklessBVH::Impl::buildPrimitivesFromBox(cuda_tool::CBufferView<AABB> aabbs)
{  ///< update idx-th _bxs to idx-th leaf
    auto k = StacklessBVH_buildPrimitivesFromBox_kernel;
    int  n = static_cast<int>(aabbs.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            ext_idx.view(), ext_aabb.view(), primMap.view(), aabbs, n);
}


UIPC_INLINE void StacklessBVH::Impl::calcExtNodeSplitMetrics()
{
    auto k = StacklessBVH_calcExtNodeSplitMetrics_kernel;
    int  n = static_cast<int>(sorted_mtcode.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            sorted_mtcode.size(), sorted_mtcode.view(), metric.view(), n);
}

UIPC_INLINE void StacklessBVH::Impl::buildIntNodes(int size)
{
    auto GridDim  = (size + 255) / 256;
    auto BlockDim = 256;

    if(GridDim > 0)
        StacklessBVH_buildIntNodes_kernel<<<GridDim, BlockDim, 0, nullptr>>>(
            size,
            // leaf nodes
            count.view(),
            ext_lca.view(),
            metric.view(),
            ext_par.view(),
            ext_mark.view(),
            ext_aabb.view(),
            // internal nodes
            int_rc.view(),
            int_lc.view(),
            int_range_y.view(),
            int_range_x.view(),
            int_mark.view(),
            int_aabb.view(),
            flags.view(),
            int_par.view());
}

UIPC_INLINE void StacklessBVH::Impl::calcIntNodeOrders(int size)
{
    auto k = StacklessBVH_calcIntNodeOrders_kernel;
    if(size > 0)
        k<<<cuda_tool::best_grid_dim(size, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            int_lc.view(),
            ext_lca.view(),
            count.view(),
            offsetTable.view(),
            tkMap.view(),
            size);
}

UIPC_INLINE void StacklessBVH::Impl::updateBvhExtNodeLinks(int size)
{
    if(flags.size() == 0)  // no internal nodes, thus no need to update
        return;

    auto k = StacklessBVH_updateBvhExtNodeLinks_kernel;
    if(size > 0)
        k<<<cuda_tool::best_grid_dim(size, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            tkMap.view(), ext_lca.view(), ext_par.view(), size);
}

UIPC_INLINE void StacklessBVH::Impl::reorderNode(int intSize)
{
    auto k = StacklessBVH_reorderNode_kernel;
    int  n = intSize + 1;
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            intSize,
            // leaf nodes
            ext_lca.view(),
            ext_aabb.view(),
            // internal nodes
            tkMap.view(),
            int_lc.view(),
            int_mark.view(),
            int_range_y.view(),
            int_aabb.view(),
            // total nodes
            nodes.view(),
            n);
}

inline void StacklessBVH::Impl::build(cuda_tool::CBufferView<AABB> aabbs)
{
    objs         = aabbs;
    auto numObjs = aabbs.size();

    if(aabbs.size() == 0)
        return;

    const unsigned int numInternalNodes = numObjs - 1;  // Total number of internal nodes
    const unsigned int numNodes = numObjs * 2 - 1;  // Total number of nodes


    mtcode.resize(numObjs);
    sorted_mtcode.resize(numObjs);
    sorted_id.resize(numObjs);
    primMap.resize(numObjs);
    ext_aabb.resize(numObjs);
    ext_idx.resize(numObjs);
    ext_lca.resize(numObjs + 1);
    ext_par.resize(numObjs);
    ext_mark.resize(numObjs);

    metric.resize(numObjs);
    tkMap.resize(numObjs);
    offsetTable.resize(numObjs);
    count.resize(numObjs);

    flags.resize(numInternalNodes);
    int_lc.resize(numInternalNodes);
    int_rc.resize(numInternalNodes);
    int_par.resize(numInternalNodes);
    int_range_x.resize(numInternalNodes);
    int_range_y.resize(numInternalNodes);
    int_mark.resize(numInternalNodes);
    int_aabb.resize(numInternalNodes);

    nodes.resize(numNodes);


    auto init = StacklessBVH_initializeBuildState_kernel;
    auto n    = static_cast<int>(numObjs);
    init<<<cuda_tool::best_grid_dim(n + 1, init), cuda_tool::best_block_dim(init), 0, nullptr>>>(
        n,
        scene_box.viewer(),
        flags.view(),
        ext_mark.view(),
        ext_lca.view(),
        ext_par.view(),
        count.view(),
        primMap.view());

    calcMaxBVFromBox(aabbs, scene_box.view());

    calcMCsFromBox(aabbs, scene_box.view(), mtcode.view());

    // CUB scratch is cached per stream by cuda_tool and grows only on demand.
    cuda_tool::DeviceRadixSort().SortPairs(
        mtcode.data(), sorted_mtcode.data(), primMap.data(), sorted_id.data(), n);

    calcInverseMapping();

    buildPrimitivesFromBox(aabbs);

    calcExtNodeSplitMetrics();

    buildIntNodes(numObjs);

    cuda_tool::DeviceScan().ExclusiveSum(count.data(), offsetTable.data(), n);

    calcIntNodeOrders(numObjs);

    updateBvhExtNodeLinks(numObjs);

    reorderNode(numInternalNodes);
}

template <typename Pred>
void StacklessBVH::Impl::StacklessCDSharedSelf(Pred                    pred,
                                               cuda_tool::VarView<int> cpNum,
                                               cuda_tool::BufferView<Vector2i> buffer)
{
    using namespace culbvh;

    auto numQuery = static_cast<int>(ext_aabb.size());
    auto numObjs  = numQuery;
    auto BlockDim = K_THREADS;
    auto GridDim  = (numQuery + BlockDim - 1) / BlockDim;

    if(GridDim > 0)
        StacklessBVH_StacklessCDSharedSelf_kernel<Pred>
            <<<GridDim, BlockDim, 0, nullptr>>>(numQuery,
                                                objs,
                                                numObjs - 1,
                                                numObjs,
                                                ext_idx.view(),
                                                nodes.view(),
                                                cpNum.viewer(),
                                                buffer,
                                                pred);
}

template <typename Pred>
void StacklessBVH::Impl::StacklessCDSharedOther(Pred pred,
                                                cuda_tool::CBufferView<AABB> query_aabbs,
                                                cuda_tool::CBufferView<int> query_sorted_id,
                                                cuda_tool::VarView<int> cpNum,
                                                cuda_tool::BufferView<Vector2i> buffer)
{
    using namespace culbvh;

    auto numQuery = static_cast<int>(query_aabbs.size());
    auto numObjs  = static_cast<int>(ext_aabb.size());
    auto BlockDim = K_THREADS;
    auto GridDim  = (numQuery + BlockDim - 1) / BlockDim;


    if(GridDim > 0)
        StacklessBVH_StacklessCDSharedOther_kernel<Pred>
            <<<GridDim, BlockDim, 0, nullptr>>>(numQuery,
                                                query_aabbs,
                                                query_sorted_id,
                                                numObjs - 1,
                                                numObjs,
                                                ext_idx.view(),
                                                nodes.view(),
                                                cpNum.viewer(),
                                                buffer,
                                                pred);
}

inline void StacklessBVH::build(cuda_tool::CBufferView<AABB> aabbs)
{
    m_impl.build(aabbs);
}

template <std::invocable<IndexT, IndexT> Pred>
void StacklessBVH::detect(Pred callback, QueryBuffer& qbuffer)
{
    using namespace cuda_tool;
    // Query the LBVH

    if(m_impl.objs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    auto do_query = [&]
    {
        // clear counter
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);

        m_impl.StacklessCDSharedSelf(
            callback, qbuffer.m_cpNum.view(), qbuffer.m_pairs.view());
    };

    do_query();

    // get total number of pairs
    int h_cp_num = qbuffer.m_cpNum;
    // if failed, resize and retry
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_impl.config.reserve_ratio);
        do_query();
    }

    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}

inline void StacklessBVH::QueryBuffer::build(cuda_tool::CBufferView<AABB> aabbs)
{
    auto size = aabbs.size();
    m_queryMtCode.resize(size);
    m_querySortedMtCode.resize(size);
    m_queryId.resize(size);
    m_querySortedId.resize(size);

    auto init = StacklessBVH_initializeQueryState_kernel;
    auto n    = static_cast<int>(size);
    if(n > 0)
        init<<<cuda_tool::best_grid_dim(n, init), cuda_tool::best_block_dim(init), 0, nullptr>>>(
            n, m_querySceneBox.viewer(), m_queryId.view());
    Impl::calcMaxBVFromBox(aabbs, m_querySceneBox);
    Impl::calcMCsFromBox(aabbs, m_querySceneBox, m_queryMtCode);
    cuda_tool::DeviceRadixSort().SortPairs(m_queryMtCode.data(),
                                           m_querySortedMtCode.data(),
                                           m_queryId.data(),
                                           m_querySortedId.data(),
                                           n);
}

template <std::invocable<IndexT, IndexT> Pred>
void StacklessBVH::query(cuda_tool::CBufferView<AABB> aabbs, Pred callback, QueryBuffer& qbuffer)
{
    if(aabbs.size() == 0 || m_impl.objs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    using namespace cuda_tool;
    qbuffer.build(aabbs);

    auto do_query = [&]
    {
        // clear counter
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);

        m_impl.StacklessCDSharedOther(callback,
                                      aabbs,
                                      qbuffer.m_querySortedId.view(),
                                      qbuffer.m_cpNum.view(),
                                      qbuffer.m_pairs.view());
    };

    do_query();

    // get total number of pairs
    int h_cp_num = qbuffer.m_cpNum;
    // if failed, resize and retry
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_impl.config.reserve_ratio);
        do_query();
    }

    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}
}  // namespace uipc::backend::cuda
