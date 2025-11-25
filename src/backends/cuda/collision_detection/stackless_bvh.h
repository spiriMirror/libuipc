#pragma once
#include <uipc/common/logger.h>
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <muda/buffer.h>

#include <thrust/device_vector.h>
#include <thrust/swap.h>
#include <thrust/sequence.h>
#include <thrust/functional.h>
#include <thrust/sort.h>
#include <thrust/fill.h>
#include <thrust/reduce.h>
#include <thrust/execution_policy.h>

namespace uipc::backend::cuda
{
/**
 * @brief Friend class to access private members of StacklessBVH (for internal use only)
 */
template <typename T>
class StacklessBVHFriend;

/**
 * @brief Stackless Bounding Volume Hierarchy for AABB overlap detection
 */
class StacklessBVH
{
  public:
    template <typename T>
    friend class StacklessBVHFriend;

    class Config
    {
      public:
        Float reserve_ratio = 1.5f;
    };

    class QueryBuffer
    {
      public:
        QueryBuffer()
        {
            m_pairs.resize(50 * 1024);  // initial capacity
        }
        auto  view() const noexcept { return m_pairs.view(0, m_size); }
        void  reserve(size_t size) { m_pairs.resize(size); }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }


      public:
        friend class StacklessBVH;
        SizeT                        m_size = 0;
        muda::DeviceBuffer<Vector2i> m_pairs;

        muda::DeviceBuffer<unsigned int> m_queryMtCode;
        muda::DeviceVar<AABB>            m_querySceneBox;
        muda::DeviceBuffer<int>          m_querySortedId;
        muda::DeviceVar<int>             m_cpNum;

        void  build(muda::CBufferView<AABB> aabbs);
        SizeT query_count() { return m_queryMtCode.size(); }
    };

    struct /*__align__(16) */ Node
    {
        IndexT lc;
        IndexT escape;
        AABB   bound;
    };

    struct DefaultQueryCallback
    {
        MUDA_GENERIC bool operator()(IndexT i, IndexT j) { return true; }
    };

    StacklessBVH(Config config = Config{})
        : m_config(config)
    {
    }

    ~StacklessBVH() = default;

    /**
     * @brief Build the Stackless BVH from given AABBs
     * 
     * 
     * @param aabbs Input AABBs, aabbs must be kept valid during the lifetime of this BVH
     */
    void build(muda::CBufferView<AABB> aabbs);

    /**
     * @brief Detect overlapping AABB pairs in the BVH
     * 
     * 
     * @param callback f: (int i, int j) -> bool Callback predicate to filter overlapping pairs
     * @param qbuffer Output buffer to store detected overlapping pairs
     */
    template <std::invocable<IndexT, IndexT> Pred = DefaultQueryCallback>
    void detect(Pred callback, QueryBuffer& qbuffer);


    /*
    * @brief Query overlapping AABBs from external AABBs
    * 
    * @param aabbs Input external AABBs to query, aabbs must be kept valid during the lifetime of this BVH
    * @param callback f: (int i, int j) -> bool Callback predicate to filter overlapping pairs
    * @param qbuffer Output buffer to store detected overlapping pairs
    */
    template <std::invocable<IndexT, IndexT> Pred = DefaultQueryCallback>
    void query(muda::CBufferView<AABB> aabbs, Pred callback, QueryBuffer& qbuffer);


  public:
    thrust::device_ptr<const AABB> d_objs = nullptr;

    muda::DeviceVar<AABB>        d_scene_box;  ///< external bounding boxes
    muda::DeviceVector<uint32_t> d_flags;
    muda::DeviceVector<uint32_t> d_mtcode;  ///< external morton codes
    muda::DeviceVector<int32_t>  d_sorted_id;
    muda::DeviceVector<int32_t>  d_primMap;
    muda::DeviceVector<int>      d_metric;
    muda::DeviceVector<uint32_t> d_count;
    muda::DeviceVector<int>      d_tkMap;
    muda::DeviceVector<uint32_t> d_offsetTable;

    muda::DeviceVector<AABB>     d_ext_aabb;
    muda::DeviceVector<int>      d_ext_idx;
    muda::DeviceVector<int>      d_ext_lca;
    muda::DeviceVector<uint32_t> d_ext_mark;
    muda::DeviceVector<uint32_t> d_ext_par;

    muda::DeviceVector<int>      d_int_lc;
    muda::DeviceVector<int>      d_int_rc;
    muda::DeviceVector<int>      d_int_par;
    muda::DeviceVector<int>      d_int_range_x;
    muda::DeviceVector<int>      d_int_range_y;
    muda::DeviceVector<uint32_t> d_int_mark;
    muda::DeviceVector<AABB>     d_int_aabb;

    muda::DeviceVector<ulonglong2> d_quantNode;

    muda::DeviceVector<Node> d_nodes;


    Config m_config;

    SizeT object_count() { return d_sorted_id.size(); }
};

}  // namespace uipc::backend::cuda

/******************************************************************************
* Implementation
******************************************************************************/

#include <cuda_device/builtin.h>
#include <collision_detection/culbvh/vector_type_t.h>
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

MUDA_INLINE void calcMaxBVFromBox(muda::CBufferView<aabb> aabbs, muda::VarView<aabb> scene_box)
{
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
                    *_bv = aabb();
                }

                __shared__ aabb aabbData[K_THREADS >> 5];

                aabb temp = box(idx);
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

MUDA_INLINE void calcMCsFromBox(muda::CBufferView<aabb>    aabbs,
                                muda::CVarView<aabb>       scene_box,
                                muda::BufferView<uint32_t> codes)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(aabbs.size(),
               [box   = aabbs.viewer().name("box"),
                scene = scene_box.viewer().name("scene"),
                codes = codes.viewer().name("codes")] __device__(int idx)
               {
                   aabb bv = box(idx);

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
MUDA_INLINE void calcInverseMapping(uipc::backend::cuda::StacklessBVH& bvh)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(bvh.d_sorted_id.size(),
               [map = bvh.d_sorted_id.viewer().name("map"),
                invMap = bvh.d_primMap.viewer().name("invMap")] __device__(int idx)
               {
                   //
                   invMap(map(idx)) = idx;
               });
}

MUDA_INLINE void buildPrimitivesFromBox(muda::CBufferView<aabb> aabbs,
                                        uipc::backend::cuda::StacklessBVH& bvh)
{  ///< update idx-th _bxs to idx-th leaf
    using namespace muda;
    ParallelFor().apply(aabbs.size(),
                        [_primIdx = bvh.d_ext_idx.viewer().name("primIdx"),
                         _primBox = bvh.d_ext_aabb.viewer().name("primBox"),
                         _primMap = bvh.d_primMap.viewer().name("primMap"),
                         box = aabbs.viewer().name("box")] __device__(int idx)
                        {
                            int  newIdx      = _primMap(idx);
                            aabb bv          = box(idx);
                            _primIdx(newIdx) = idx;
                            _primBox(newIdx) = bv;
                        });
}


MUDA_INLINE void calcExtNodeSplitMetrics(uipc::backend::cuda::StacklessBVH& bvh)
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(bvh.d_mtcode.size(),
               [extsize = bvh.d_mtcode.size(),
                _codes  = bvh.d_mtcode.viewer().name("_codes"),
                _metrics = bvh.d_metric.viewer().name("_metrics")] __device__(int idx)
               {
                   _metrics(idx) = idx != extsize - 1 ?
                                       32 - __clz(_codes(idx) ^ _codes(idx + 1)) :
                                       33;
               });
}

MUDA_INLINE void buildIntNodes(int                                size,
    uipc::backend::cuda::StacklessBVH& bvh
                               /*
                               uint32_t*                          _depths,
                               int*                               _lvs_lca,
                               int*                               _lvs_metric,
                               uint32_t*                          _lvs_par,
                               uint32_t*                          _lvs_mark,
                               aabb*                              _lvs_box,
                               int*                               _tks_rc,
                               int*                               _tks_lc,
                               int*                               _tks_range_y,
                               int*                               _tks_range_x,
                               uint32_t*                          _tks_mark,
                               aabb*                              _tks_box,
                               uint32_t*                          _flag,
                               int*                               _tks_par*/)
{
    using namespace muda;

    auto GridDim  = (size + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .file_line(__FILE__, __LINE__)
        .apply(
            [size         = size,
             _depths      = bvh.d_count.viewer().name("_depths"),
             _lvs_lca     = bvh.d_ext_lca.viewer().name("_lvs_lca"),
             _lvs_metric  = bvh.d_metric.viewer().name("_lvs_metric"),
             _lvs_par     = bvh.d_ext_par.viewer().name("_lvs_par"),
             _lvs_mark    = bvh.d_ext_mark.viewer().name("_lvs_mark"),
             _lvs_box     = bvh.d_ext_aabb.viewer().name("_lvs_box"),
             _tks_rc      = bvh.d_int_rc.viewer().name("_tks_rc"),
             _tks_lc      = bvh.d_int_lc.viewer().name("_tks_lc"),
             _tks_range_y = bvh.d_int_range_y.viewer().name("_tks_range_y"),
             _tks_range_x = bvh.d_int_range_x.viewer().name("_tks_range_x"),
             _tks_mark    = bvh.d_int_mark.viewer().name("_tks_mark"),
             _tks_box     = bvh.d_int_aabb.viewer().name("_tks_box"),
             _flag        = bvh.d_flags.viewer().name("_flag"),
             _tks_par = bvh.d_int_par.viewer().name("_tks_par")] __device__()
            {
                int idx = blockIdx.x * blockDim.x + threadIdx.x;
                if(idx >= size)
                    return;
                //_tks_range_x[idx] = -1;
                //__syncthreads();

                _lvs_lca(idx) = -1, _depths(idx) = 0;
                int  l = idx - 1, r = idx;  ///< (l, r]
                bool mark;
                if(l >= 0)
                    mark = _lvs_metric(l) < _lvs_metric(r);  //determine direction
                else
                    mark = false;
                int cur = mark ? l : r;

                //if (cur == 254)printf("%d  %d  %d  %d  %d\n", idx, mark, _lvs_metric[l], _lvs_metric[r], cur);
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
                    //if (idx < 10) printf("%d   %d\n", idx, _tks_mark[cur]);
                    //if (_tks_range_x[cur] == 0) printf("cur:%d  %d  %d   %d   %d\n", cur, _tks_range_x[252], _tks_range_y[252], _tks_lc[252], _tks_rc[252]);
                    l               = _tks_range_x(cur) - 1;
                    r               = _tks_range_y(cur);
                    _lvs_lca(l + 1) = cur /*, _tks.rcd(cur) = ++_lvs.rcl(r)*/;
                    _depths(l + 1)++;
                    if(l >= 0)
                        mark = _lvs_metric(l) < _lvs_metric(r);  ///< true when right child, false otherwise
                    else
                        mark = false;

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

MUDA_INLINE void calcIntNodeOrders(int size, uipc::backend::cuda::StacklessBVH& bvh
                                   /*int* _tks_lc, int* _lcas, uint32_t* _depths, uint32_t* _offsets, int* _tkMap*/)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(size,
               [_tks_lc  = bvh.d_int_lc.viewer().name("_tks_lc"),
                _lcas    = bvh.d_ext_lca.viewer().name("_lcas"),
                _depths  = bvh.d_count.viewer().name("_depths"),
                _offsets = bvh.d_offsetTable.viewer().name("_offsets"),
                _tkMap = bvh.d_tkMap.viewer().name("_tkMap")] __device__(int idx)
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

MUDA_INLINE void updateBvhExtNodeLinks(int size, uipc::backend::cuda::StacklessBVH& bvh
                                       /*const int* _mapTable, int* _lcas, uint* _pars*/)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(size,
               [_mapTable = bvh.d_tkMap.viewer().name("_mapTable"),
                _lcas     = bvh.d_ext_lca.viewer().name("_lcas"),
                _pars = bvh.d_ext_par.viewer().name("_pars")] __device__(int idx)
               {
                   int ori;
                   _pars(idx) = _mapTable(_pars(idx));
                   if((ori = _lcas(idx)) != -1)
                       _lcas(idx) = _mapTable(ori) << 1;
                   else
                       _lcas(idx) = idx << 1 | 1;
               });
}

MUDA_INLINE void reorderNode(int            intSize,
     uipc::backend::cuda::StacklessBVH& bvh
    
    /*
                             const int*     _tkMap,
                             int*           _lvs_lca,
                             aabb*          _lvs_box,
                             int*           _unorderedTks_lc,
                             uint32_t*      _unorderedTks_mark,
                             int*           _unorderedTks_rangey,
                             aabb*          _unorderedTks_box,
                             stacklessnode* _nodes*/)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(intSize + 1,
               [intSize,
                _tkMap   = bvh.d_tkMap.viewer().name("_tkMap"),
                _lvs_lca = bvh.d_ext_lca.viewer().name("_lvs_lca"),
                _lvs_box = bvh.d_ext_aabb.viewer().name("_lvs_box"),
                _unorderedTks_lc = bvh.d_int_lc.viewer().name("_unorderedTks_lc"),
                _unorderedTks_mark = bvh.d_int_mark.viewer().name("_unorderedTks_mark"),
                _unorderedTks_rangey = bvh.d_int_range_y.viewer().name("_unorderedTks_rangey"),
                _unorderedTks_box = bvh.d_int_aabb.viewer().name("_unorderedTks_box"),
                _nodes = bvh.d_nodes.viewer().name("_nodes")] __device__(int idx)
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

MUDA_GENERIC MUDA_INLINE bool overlapsLonglong2int(const ulonglong2& a, const intAABB& b)
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
    //return int2{a, b};
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

    auto CopyCount = min(totalResInBlock, maxRes - globalIdx);

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

template <typename Pred>
MUDA_INLINE void StacklessCDSharedSelf(Pred                 pred,
                                       uint                 Size,
                                       const aabb*          _box,
                                       const int            intSize,
                                       const int*           _lvs_idx,
                                       const stacklessnode* _nodes,
                                       int*                 resCounter,
                                       Vector2i*            res,
                                       const int            maxRes)
{
    using namespace muda;

    auto numQuery = Size;
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .apply(
            [=] __device__()
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

                const int MaxIter = 1000;
                int       iter    = 0;
                for(; iter < 1000; iter++)
                {
                    __syncthreads();
                    if(active)
                    {
                        while(st != -1)
                        {
                            // Load node data - Eigen::AlignedBox stores min and max as Vector3f members
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
                                        auto pair =
                                            make_ordered_pair(idx, _lvs_idx[st - intSize]);
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
                    }
                    // Flush whatever we have
                    __syncthreads();
                    __threadfence();
                    int totalResInBlock = min(sharedCounter, MAX_RES_PER_BLOCK);

                    if(threadIdx.x == 0)
                    {
                        // This Block Starts writing at sharedGlobalIdx
                        sharedGlobalIdx = atomicAdd(resCounter, totalResInBlock);
                    }

                    __syncthreads();

                    // Make sure we dont write out of bounds
                    const int globalIdx = sharedGlobalIdx;

                    if(threadIdx.x == 0)
                        sharedCounter = 0;

                    // if there is at least one element empty
                    // it means we have found all collisions for this block
                    bool done = totalResInBlock < MAX_RES_PER_BLOCK;

                    SafeCopyTo(sharedRes, totalResInBlock, res, globalIdx, maxRes);

                    if(done)
                        break;

                    if(iter >= MaxIter - 3)
                    {
                        auto max_res_per_block = MAX_RES_PER_BLOCK;
                        MUDA_KERNEL_PRINT("[%d] Exceeded max iteration in stackless traversal, totalResInBlock=%d, MAX_RES_PER_BLOCK=%d\n",
                                          iter,
                                          totalResInBlock,
                                          max_res_per_block);
                    }
                    if(iter == MaxIter - 1)
                    {
                        MUDA_ASSERT(false, "Exceeded max iteration in stackless traversal");
                    }
                }
            });
}

template <typename Pred>
__global__ void KStacklessCDSharedOther(Pred                 pred,
                                        uint                 Size,
                                        const aabb*          _box,
                                        const int*           sortedIdx,
                                        const int            intSize,
                                        const int*           _lvs_idx,
                                        const stacklessnode* _nodes,
                                        int*                 resCounter,
                                        Vector2i*            res,
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
    const int     MaxIter = intSize * 2 + 1;
    int           iter    = 0;
    for(; iter < 1000; iter++)
    {
        __syncthreads();
        if(active)
        {

            auto inner_I = 0;
            for(; inner_I < MaxIter; inner_I++)
            {
                if(st == -1)
                    break;

                node.lc     = _nodes[st].lc;
                node.escape = _nodes[st].escape;
                node.bound  = _nodes[st].bound;

                //node = _nodes[st];
                if(node.bound.intersects(bv))
                {
                    if(node.lc == -1)
                    {
                        auto pair = int2{idx, _lvs_idx[st - intSize]};
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

                if(inner_I >= MaxIter - 10)
                {
                    MUDA_KERNEL_PRINT("[%d][%d] Exceeded max iteration in stackless traversal, st=%d, node.lc=%d, node.escape=%d\n",
                                      iter,
                                      inner_I,
                                      st,
                                      node.lc,
                                      node.escape);
                }
            }
            if(inner_I == MaxIter)
            {
                MUDA_ASSERT(false, "Exceeded max iteration in stackless traversal, %d", inner_I);
            }
        }
        // Flush whatever we have
        __syncthreads();
        int totalResInBlock = min(sharedCounter, MAX_RES_PER_BLOCK);

        if(threadIdx.x == 0)
        {
            // This Block Starts writing at sharedGlobalIdx
            sharedGlobalIdx = atomicAdd(resCounter, totalResInBlock);
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

        SafeCopyTo(sharedRes, totalResInBlock, res, globalIdx, maxRes);

        if(done)
            break;


        if(iter >= MaxIter - 3)
        {
            auto max_res_per_block = MAX_RES_PER_BLOCK;
            MUDA_KERNEL_PRINT("[%d] Exceeded max iteration in stackless traversal, totalResInBlock=%d, MAX_RES_PER_BLOCK=%d\n",
                              iter,
                              totalResInBlock,
                              max_res_per_block);
        }
        if(iter == MaxIter - 1)
        {
            MUDA_ASSERT(false, "Exceeded max iteration in stackless traversal");
        }
    }
}

template <typename Pred>
MUDA_INLINE void StacklessCDSharedOther(Pred                 pred,
                                        uint                 Size,
                                        const aabb*          _box,
                                        const int*           sortedIdx,
                                        const int            intSize,
                                        const int*           _lvs_idx,
                                        const stacklessnode* _nodes,
                                        int*                 resCounter,
                                        Vector2i*            res,
                                        const int            maxRes)
{
    using namespace muda;

    auto numQuery = Size;
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    KStacklessCDSharedOther<<<GridDim, BlockDim>>>(
        pred, Size, _box, sortedIdx, intSize, _lvs_idx, _nodes, resCounter, res, maxRes);
}
}  // namespace uipc::culbvh

namespace uipc::backend::cuda
{
inline void StacklessBVH::build(muda::CBufferView<AABB> aabbs)
{
    auto devicePtr = aabbs.data();
    d_objs         = thrust::device_ptr<const AABB>(devicePtr);
    auto numObjs   = aabbs.size();

    const unsigned int numInternalNodes = numObjs - 1;  // Total number of internal nodes
    const unsigned int numNodes = numObjs * 2 - 1;  // Total number of nodes

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
    d_nodes.resize(numNodes);


    // Initialize flags to 0
    thrust::fill(d_flags.begin(), d_flags.end(), 0);
    thrust::fill(thrust::device, d_ext_mark.begin(), d_ext_mark.end(), 7);
    thrust::fill(thrust::device, d_ext_lca.begin(), d_ext_lca.end(), 0);
    thrust::fill(thrust::device, d_ext_par.begin(), d_ext_par.end(), 0);

    culbvh::calcMaxBVFromBox(aabbs, d_scene_box.view());

    culbvh::calcMCsFromBox(aabbs, d_scene_box.view(), d_mtcode.view());

    auto null_stream = thrust::cuda::par.on(nullptr);

    thrust::sequence(null_stream, d_sorted_id.begin(), d_sorted_id.end());
    thrust::sort_by_key(
        null_stream, d_mtcode.begin(), d_mtcode.end(), d_sorted_id.begin());


    culbvh::calcInverseMapping(*this);

    culbvh::buildPrimitivesFromBox(aabbs, *this);

    culbvh::calcExtNodeSplitMetrics(*this);

    culbvh::buildIntNodes(numObjs, *this);

    thrust::exclusive_scan(
        null_stream, d_count.begin(), d_count.end(), d_offsetTable.begin());


    culbvh::calcIntNodeOrders(numObjs, *this);

    thrust::fill(null_stream, d_ext_lca.begin() + numObjs, d_ext_lca.begin() + numObjs + 1, -1);

    culbvh::updateBvhExtNodeLinks(numObjs, *this);

    culbvh::reorderNode(numInternalNodes, *this);
}

template <std::invocable<IndexT, IndexT> Pred>
void StacklessBVH::detect(Pred callback, QueryBuffer& qbuffer)
{
    using namespace muda;
    // Query the LBVH
    const culbvh::uint numQuery = d_ext_aabb.size();
    const int          numObjs  = d_ext_aabb.size();

    auto do_query = [&]
    {
        auto d_cpNum   = qbuffer.m_cpNum.data();
        auto d_cpRes   = qbuffer.m_pairs.data();
        auto max_cpNum = qbuffer.m_pairs.size();

        // clear counter
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);

        culbvh::StacklessCDSharedSelf(callback,
                                      numQuery,
                                      thrust::raw_pointer_cast(d_objs),
                                      numObjs - 1,
                                      thrust::raw_pointer_cast(d_ext_idx.data()),
                                      thrust::raw_pointer_cast(d_nodes.data()),
                                      d_cpNum,
                                      d_cpRes,
                                      max_cpNum);
    };

    do_query();

    // get total number of pairs
    int h_cp_num = qbuffer.m_cpNum;
    // if failed, resize and retry
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_config.reserve_ratio);
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

    auto devicePtr       = aabbs.data();
    auto d_querySceneBox = m_querySceneBox.data();
    auto d_queryMtCode   = m_queryMtCode.data();
    auto d_querySortedId = m_querySortedId.data();

    auto numQuery = size;
    culbvh::calcMaxBVFromBox(aabbs, m_querySceneBox);

    culbvh::calcMCsFromBox(aabbs, m_querySceneBox, m_queryMtCode);

    auto null_stream = thrust::cuda::par.on(nullptr);

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
    muda::wait_device();

    auto numQuery        = static_cast<culbvh::uint>(qbuffer.query_count());
    auto numObjs         = static_cast<int>(object_count());
    auto devicePtr       = aabbs.data();
    auto d_querySortedId = qbuffer.m_querySortedId.data();

    auto do_query = [&]
    {
        auto d_cpNum   = qbuffer.m_cpNum.data();
        auto d_cpRes   = qbuffer.m_pairs.data();
        auto max_cpNum = qbuffer.m_pairs.size();

        // clear counter
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);

        culbvh::StacklessCDSharedOther(callback,
                                       numQuery,
                                       devicePtr,
                                       d_querySortedId,
                                       numObjs - 1,
                                       thrust::raw_pointer_cast(d_ext_idx.data()),
                                       thrust::raw_pointer_cast(d_nodes.data()),
                                       d_cpNum,
                                       d_cpRes,
                                       max_cpNum);
    };

    do_query();

    // get total number of pairs
    int h_cp_num = qbuffer.m_cpNum;
    // if failed, resize and retry
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_config.reserve_ratio);
        do_query();
    }

    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}
}  // namespace uipc::backend::cuda