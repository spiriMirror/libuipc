#include <cuda_device/builtin.h>
#include <muda/launch.h>

namespace uipc::culbvh
{
using aabb          = uipc::backend::cuda::AABB;
using stacklessnode = uipc::backend::cuda::StacklessBVH::Node;
using Vector2i      = uipc::Vector2i;

using uint                       = uint32_t;
using ullint                     = unsigned long long int;
constexpr int K_THREADS          = 256;
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

struct intAABB
{
    int3 _min, _max;

    MUDA_GENERIC MUDA_INLINE void convertFrom(const aabb& other, float3& origin, float3& delta)
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
MUDA_GENERIC MUDA_INLINE T __mm_min(T a, T b)
{
    return a > b ? b : a;
}

template <typename T>
MUDA_GENERIC MUDA_INLINE T __mm_max(T a, T b)
{
    return a > b ? a : b;
}

MUDA_DEVICE MUDA_INLINE float atomicMinf(float* addr, float value)
{
    float old;
    old = (value >= 0) ?
              __int_as_float(atomicMin((int*)addr, __float_as_int(value))) :
              __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));
    return old;
}

MUDA_DEVICE MUDA_INLINE float atomicMaxf(float* addr, float value)
{
    float old;
    old = (value >= 0) ?
              __int_as_float(atomicMax((int*)addr, __float_as_int(value))) :
              __uint_as_float(atomicMin((unsigned int*)addr, __float_as_uint(value)));
    return old;
}

MUDA_GENERIC MUDA_INLINE uint expandBits(uint v)
{  ///< Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

MUDA_GENERIC MUDA_INLINE uint morton3D(float x, float y, float z)
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
MUDA_GENERIC MUDA_INLINE bool lessThan(const int3& a, const int3& b)
{
    if(a.x != b.x)
        return a.x < b.x;
    if(a.y != b.y)
        return a.y < b.y;
    return a.z < b.z;
}

MUDA_GENERIC MUDA_INLINE Vector2i to_eigen(int2 v)
{
    return Vector2i{v.x, v.y};
}

MUDA_GENERIC MUDA_INLINE int2 make_ordered_pair(int a, int b)
{
    if(a < b)
        return int2{a, b};
    else
        return int2{b, a};
}

MUDA_GENERIC MUDA_INLINE float3 operator-(const float3& v0, const float3& v1)
{
    return make_float3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

MUDA_GENERIC MUDA_INLINE void SafeCopyTo(int2* sharedRes,
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
MUDA_INLINE void StacklessBVH::Impl::calcMaxBVFromBox(muda::CBufferView<AABB> aabbs,
                                                      muda::VarView<AABB> scene_box)
{
    using namespace culbvh;

    auto numQuery = aabbs.size();
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    using namespace muda;

    Launch(GridDim, BlockDim)
        .file_line(__FILE__, __LINE__)
        .apply(
            [size = aabbs.size(),
             box  = aabbs.viewer().name("box"),
             _bv  = scene_box.viewer().name("_bv")] __device__()
            {
                int idx     = blockIdx.x * blockDim.x + threadIdx.x;
                int warpTid = threadIdx.x % 32;
                int warpId  = (threadIdx.x >> 5);
                int warpNum;
                if(idx >= size)
                    return;
                if(idx == 0)
                {
                    *_bv = AABB();
                }

                __shared__ AABB aabbData[K_THREADS >> 5];

                AABB temp = box(idx);
                __syncthreads();

                // Extract values for warp shuffle
                float tempMinX = temp.min().x();
                float tempMinY = temp.min().y();
                float tempMinZ = temp.min().z();
                float tempMaxX = temp.max().x();
                float tempMaxY = temp.max().y();
                float tempMaxZ = temp.max().z();

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
                    aabbData[warpId].min() = Eigen::Vector3f(tempMinX, tempMinY, tempMinZ);
                    aabbData[warpId].max() = Eigen::Vector3f(tempMaxX, tempMaxY, tempMaxZ);
                }
                __syncthreads();
                if(threadIdx.x >= warpNum)
                    return;

                if(warpNum > 1)
                {
                    temp     = aabbData[threadIdx.x];
                    tempMinX = temp.min().x();
                    tempMinY = temp.min().y();
                    tempMinZ = temp.min().z();
                    tempMaxX = temp.max().x();
                    tempMaxY = temp.max().y();
                    tempMaxZ = temp.max().z();

                    for(int i = 1; i < warpNum; i = (i << 1))
                    {
                        float otherMinX = __shfl_down_sync(0xffffffff, tempMinX, i);
                        float otherMinY = __shfl_down_sync(0xffffffff, tempMinY, i);
                        float otherMinZ = __shfl_down_sync(0xffffffff, tempMinZ, i);
                        float otherMaxX = __shfl_down_sync(0xffffffff, tempMaxX, i);
                        float otherMaxY = __shfl_down_sync(0xffffffff, tempMaxY, i);
                        float otherMaxZ = __shfl_down_sync(0xffffffff, tempMaxZ, i);
                        tempMinX = __mm_min(tempMinX, otherMinX);
                        tempMinY = __mm_min(tempMinY, otherMinY);
                        tempMinZ = __mm_min(tempMinZ, otherMinZ);
                        tempMaxX = __mm_max(tempMaxX, otherMaxX);
                        tempMaxY = __mm_max(tempMaxY, otherMaxY);
                        tempMaxZ = __mm_max(tempMaxZ, otherMaxZ);
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
            });
}

MUDA_INLINE void StacklessBVH::Impl::calcMCsFromBox(muda::CBufferView<AABB> aabbs,
                                                    muda::CVarView<AABB> scene_box,
                                                    muda::BufferView<uint32_t> codes)
{
    using namespace culbvh;
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(aabbs.size(),
               [box   = aabbs.viewer().name("box"),
                scene = scene_box.viewer().name("scene"),
                codes = codes.viewer().name("codes")] __device__(int idx)
               {
                   AABB bv = box(idx);

                   // Get center using Eigen API
                   auto   center = bv.center();
                   float3 c = make_float3(center.x(), center.y(), center.z());

                   // Get scene min
                   auto   sceneMin = scene->min();
                   float3 sceneMinVec =
                       make_float3(sceneMin.x(), sceneMin.y(), sceneMin.z());
                   const float3 offset = c - sceneMinVec;

                   // Get dimensions
                   auto sceneSize = scene->sizes();
                   codes(idx)     = morton3D(offset.x / sceneSize.x(),
                                         offset.y / sceneSize.y(),
                                         offset.z / sceneSize.z());
               });
}

/// incoherent access, thus poor performance
MUDA_INLINE void StacklessBVH::Impl::calcInverseMapping()
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(sorted_id.size(),
               [map    = sorted_id.viewer().name("map"),
                invMap = primMap.viewer().name("invMap")] __device__(int idx)
               {
                   //
                   invMap(map(idx)) = idx;
               });
}

MUDA_INLINE void StacklessBVH::Impl::buildPrimitivesFromBox(muda::CBufferView<AABB> aabbs)
{  ///< update idx-th _bxs to idx-th leaf
    using namespace muda;
    ParallelFor().apply(aabbs.size(),
                        [_primIdx = ext_idx.viewer().name("primIdx"),
                         _primBox = ext_aabb.viewer().name("primBox"),
                         _primMap = primMap.viewer().name("primMap"),
                         box = aabbs.viewer().name("box")] __device__(int idx)
                        {
                            int  newIdx      = _primMap(idx);
                            AABB bv          = box(idx);
                            _primIdx(newIdx) = idx;
                            _primBox(newIdx) = bv;
                        });
}


MUDA_INLINE void StacklessBVH::Impl::calcExtNodeSplitMetrics()
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(mtcode.size(),
               [extsize  = mtcode.size(),
                _codes   = mtcode.viewer().name("_codes"),
                _metrics = metric.viewer().name("_metrics")] __device__(int idx)
               {
                   _metrics(idx) = idx != extsize - 1 ?
                                       32 - __clz(_codes(idx) ^ _codes(idx + 1)) :
                                       33;
               });
}

MUDA_INLINE void StacklessBVH::Impl::buildIntNodes(int size)
{
    using namespace muda;

    auto GridDim  = (size + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .file_line(__FILE__, __LINE__)
        .apply(
            [size         = size,
             _depths      = count.viewer().name("_depths"),
             _lvs_lca     = ext_lca.viewer().name("_lvs_lca"),
             _lvs_metric  = metric.viewer().name("_lvs_metric"),
             _lvs_par     = ext_par.viewer().name("_lvs_par"),
             _lvs_mark    = ext_mark.viewer().name("_lvs_mark"),
             _lvs_box     = ext_aabb.viewer().name("_lvs_box"),
             _tks_rc      = int_rc.viewer().name("_tks_rc"),
             _tks_lc      = int_lc.viewer().name("_tks_lc"),
             _tks_range_y = int_range_y.viewer().name("_tks_range_y"),
             _tks_range_x = int_range_x.viewer().name("_tks_range_x"),
             _tks_mark    = int_mark.viewer().name("_tks_mark"),
             _tks_box     = int_aabb.viewer().name("_tks_box"),
             _flag        = flags.viewer().name("_flag"),
             _tks_par     = int_par.viewer().name("_tks_par")] __device__()
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
            });
}

MUDA_INLINE void StacklessBVH::Impl::calcIntNodeOrders(int size)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(size,
               [_tks_lc  = int_lc.viewer().name("_tks_lc"),
                _lcas    = ext_lca.viewer().name("_lcas"),
                _depths  = count.viewer().name("_depths"),
                _offsets = offsetTable.viewer().name("_offsets"),
                _tkMap   = tkMap.viewer().name("_tkMap")] __device__(int idx)
               {
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
               });
}

MUDA_INLINE void StacklessBVH::Impl::updateBvhExtNodeLinks(int size)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(size,
               [_mapTable = tkMap.viewer().name("_mapTable"),
                _lcas     = ext_lca.viewer().name("_lcas"),
                _pars     = ext_par.viewer().name("_pars")] __device__(int idx)
               {
                   int ori;
                   _pars(idx) = _mapTable(_pars(idx));
                   if((ori = _lcas(idx)) != -1)
                       _lcas(idx) = _mapTable(ori) << 1;
                   else
                       _lcas(idx) = idx << 1 | 1;
               });
}

MUDA_INLINE void StacklessBVH::Impl::reorderNode(int intSize)
{
    using namespace culbvh;
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(intSize + 1,
               [intSize,
                _tkMap           = tkMap.viewer().name("_tkMap"),
                _lvs_lca         = ext_lca.viewer().name("_lvs_lca"),
                _lvs_box         = ext_aabb.viewer().name("_lvs_box"),
                _unorderedTks_lc = int_lc.viewer().name("_unorderedTks_lc"),
                _unorderedTks_mark = int_mark.viewer().name("_unorderedTks_mark"),
                _unorderedTks_rangey = int_range_y.viewer().name("_unorderedTks_rangey"),
                _unorderedTks_box = int_aabb.viewer().name("_unorderedTks_box"),
                _nodes = nodes.viewer().name("_nodes")] __device__(int idx)
               {
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

                   internalNode.lc = mark & 1 ? _unorderedTks_lc(idx) + intSize :
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
               });
}

inline void StacklessBVH::Impl::build(muda::CBufferView<AABB> aabbs)
{
    objs         = aabbs;
    auto numObjs = aabbs.size();

    const unsigned int numInternalNodes = numObjs - 1;  // Total number of internal nodes
    const unsigned int numNodes = numObjs * 2 - 1;  // Total number of nodes

    flags.resize(numInternalNodes);
    mtcode.resize(numObjs);
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


    int_lc.resize(numInternalNodes);
    int_rc.resize(numInternalNodes);
    int_par.resize(numInternalNodes);
    int_range_x.resize(numInternalNodes);
    int_range_y.resize(numInternalNodes);
    int_mark.resize(numInternalNodes);
    int_aabb.resize(numInternalNodes);
    nodes.resize(numNodes);


    // Initialize flags to 0
    thrust::fill(flags.begin(), flags.end(), 0);
    thrust::fill(thrust::device, ext_mark.begin(), ext_mark.end(), 7);
    thrust::fill(thrust::device, ext_lca.begin(), ext_lca.end(), 0);
    thrust::fill(thrust::device, ext_par.begin(), ext_par.end(), 0);

    calcMaxBVFromBox(aabbs, scene_box.view());

    calcMCsFromBox(aabbs, scene_box.view(), mtcode.view());

    auto null_stream = thrust::cuda::par_nosync.on(nullptr);

    thrust::sequence(null_stream, sorted_id.begin(), sorted_id.end());
    thrust::sort_by_key(null_stream, mtcode.begin(), mtcode.end(), sorted_id.begin());


    calcInverseMapping();

    buildPrimitivesFromBox(aabbs);

    calcExtNodeSplitMetrics();

    buildIntNodes(numObjs);

    thrust::exclusive_scan(null_stream, count.begin(), count.end(), offsetTable.begin());

    calcIntNodeOrders(numObjs);

    thrust::fill(null_stream, ext_lca.begin() + numObjs, ext_lca.begin() + numObjs + 1, -1);

    updateBvhExtNodeLinks(numObjs);

    reorderNode(numInternalNodes);
}

template <typename Pred>
void StacklessBVH::Impl::StacklessCDSharedSelf(Pred               pred,
                                               muda::VarView<int> cpNum,
                                               muda::BufferView<Vector2i> buffer)
{
    using namespace muda;
    using namespace culbvh;

    auto numQuery = static_cast<int>(ext_aabb.size());
    auto numObjs  = numQuery;
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .apply(
            [Size       = numQuery,
             _box       = objs.viewer().name("_box"),
             intSize    = numObjs - 1,
             numObjs    = numObjs,
             _lvs_idx   = ext_idx.viewer().name("_lvs_idx"),
             _nodes     = nodes.viewer().name("_nodes"),
             resCounter = cpNum.viewer().name("resCounter"),
             res        = buffer.viewer().name("res"),
             pred] __device__()
            {
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

                int  st = 0;
                Node node;
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
                                        auto pair =
                                            make_ordered_pair(idx, _lvs_idx(st - intSize));
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

                        MUDA_ASSERT(inner_I < MaxIter,
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
            });
}

template <typename Pred>
void StacklessBVH::Impl::StacklessCDSharedOther(Pred pred,
                                                muda::CBufferView<AABB> query_aabbs,
                                                muda::CBufferView<int> query_sorted_id,
                                                muda::VarView<int> cpNum,
                                                muda::BufferView<Vector2i> buffer)
{
    using namespace muda;
    using namespace culbvh;

    auto numQuery = static_cast<int>(query_aabbs.size());
    auto numObjs  = static_cast<int>(ext_aabb.size());
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .apply(
            [Size       = numQuery,
             _box       = query_aabbs.viewer().name("_box"),
             sortedIdx  = query_sorted_id.viewer().name("sortedIdx"),
             intSize    = numObjs - 1,
             numObjs    = numObjs,
             _lvs_idx   = ext_idx.viewer().name("_lvs_idx"),
             _nodes     = nodes.viewer().name("_nodes"),
             resCounter = cpNum.viewer().name("resCounter"),
             res        = buffer.viewer().name("res"),
             pred] __device__()
            {
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

                int  st = 0;
                Node node;

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

                        MUDA_ASSERT(inner_I < MaxIter,
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
            });
}

inline void StacklessBVH::build(muda::CBufferView<AABB> aabbs)
{
    m_impl.build(aabbs);
}

template <std::invocable<IndexT, IndexT> Pred>
void StacklessBVH::detect(Pred callback, QueryBuffer& qbuffer)
{
    using namespace muda;
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

inline void StacklessBVH::QueryBuffer::build(muda::CBufferView<AABB> aabbs)
{
    auto size = aabbs.size();
    m_queryMtCode.resize(size);
    m_querySortedId.resize(size);


    Impl::calcMaxBVFromBox(aabbs, m_querySceneBox);
    Impl::calcMCsFromBox(aabbs, m_querySceneBox, m_queryMtCode);

    auto d_querySceneBox = m_querySceneBox.data();
    auto d_queryMtCode   = m_queryMtCode.data();
    auto d_querySortedId = m_querySortedId.data();
    auto numQuery        = size;

    auto null_stream = thrust::cuda::par_nosync.on(nullptr);
    thrust::sequence(null_stream, d_querySortedId, d_querySortedId + numQuery);
    thrust::sort_by_key(null_stream, d_queryMtCode, d_queryMtCode + numQuery, d_querySortedId);
}

template <std::invocable<IndexT, IndexT> Pred>
void StacklessBVH::query(muda::CBufferView<AABB> aabbs, Pred callback, QueryBuffer& qbuffer)
{
    if(aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    using namespace muda;
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