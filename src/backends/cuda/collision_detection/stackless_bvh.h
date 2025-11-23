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
class StacklessBVH
{
  public:
    class Config
    {
      public:
        bool  quat_node     = true;
        Float reserve_ratio = 1.5f;
    };

    class QueryBuffer
    {
      public:
        QueryBuffer()
        {
            m_pairs.resize(4 * 1024);  // initial capacity
        }
        auto  view() const noexcept { return m_pairs.view(0, m_size); }
        void  reserve(size_t size) { m_pairs.resize(size); }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }


      private:
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

    struct __align__(16) Node
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

    thrust::device_vector<AABB>     d_scene_box;  ///< external bounding boxes
    thrust::device_vector<uint32_t> d_flags;
    thrust::device_vector<uint32_t> d_mtcode;  ///< external morton codes
    thrust::device_vector<int32_t>  d_sorted_id;
    thrust::device_vector<int32_t>  d_primMap;
    thrust::device_vector<int>      d_metric;
    thrust::device_vector<uint32_t> d_count;
    thrust::device_vector<int>      d_tkMap;
    thrust::device_vector<uint32_t> d_offsetTable;

    thrust::device_vector<AABB>     d_ext_aabb;
    thrust::device_vector<int>      d_ext_idx;
    thrust::device_vector<int>      d_ext_lca;
    thrust::device_vector<uint32_t> d_ext_mark;
    thrust::device_vector<uint32_t> d_ext_par;

    thrust::device_vector<int>      d_int_lc;
    thrust::device_vector<int>      d_int_rc;
    thrust::device_vector<int>      d_int_par;
    thrust::device_vector<int>      d_int_range_x;
    thrust::device_vector<int>      d_int_range_y;
    thrust::device_vector<uint32_t> d_int_mark;
    thrust::device_vector<AABB>     d_int_aabb;

    thrust::device_vector<ulonglong2> d_quantNode;

    thrust::device_vector<Node> d_nodes;


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

MUDA_INLINE void calcMaxBVFromBox(int size, const aabb* box, aabb* _bv)
{
    using namespace muda;

    ParallelFor().apply(
        size,
        [size, box = box, _bv = _bv] __device__(int idx)
        {
            int warpTid = threadIdx.x % 32;
            int warpId  = (threadIdx.x >> 5);
            int warpNum;

            if(idx == 0)
            {
                _bv[0] = aabb();
            }

            __shared__ aabb aabbData[K_THREADS >> 5];

            aabb temp = box[idx];
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
                // Use data() to get pointer to underlying array for atomic operations
                float* ptrLowerBound = _bv[0].min().data();
                float* ptrUpperBound = _bv[0].max().data();
                atomicMinf(ptrLowerBound, tempMinX);
                atomicMinf(ptrLowerBound + 1, tempMinY);
                atomicMinf(ptrLowerBound + 2, tempMinZ);
                atomicMaxf(ptrUpperBound, tempMaxX);
                atomicMaxf(ptrUpperBound + 1, tempMaxY);
                atomicMaxf(ptrUpperBound + 2, tempMaxZ);
            }
        });
}

MUDA_INLINE void calcMCsFromBox(int size, const aabb* box, aabb* scene, uint* codes)
{
    using namespace muda;

    ParallelFor().apply(size,
                        [box = box, scene = scene, codes = codes] __device__(int idx)
                        {
                            aabb bv = box[idx];

                            // Get center using Eigen API
                            auto   center = bv.center();
                            float3 c =
                                make_float3(center.x(), center.y(), center.z());

                            // Get scene min
                            auto   sceneMin = scene[0].min();
                            float3 sceneMinVec =
                                make_float3(sceneMin.x(), sceneMin.y(), sceneMin.z());
                            const float3 offset = c - sceneMinVec;

                            // Get dimensions
                            auto sceneSize = scene[0].sizes();
                            codes[idx]     = morton3D(offset.x / sceneSize.x(),
                                                  offset.y / sceneSize.y(),
                                                  offset.z / sceneSize.z());
                        });
}

/// incoherent access, thus poor performance
MUDA_INLINE void calcInverseMapping(int size, int* map, int* invMap)
{
    using namespace muda;

    ParallelFor().apply(size,
                        [map = map, invMap = invMap] __device__(int idx)
                        {
                            //
                            invMap[map[idx]] = idx;
                        });
}

MUDA_INLINE void buildPrimitivesFromBox(
    int size, int* _primIdx, aabb* _primBox, int* _primMap, const aabb* box)
{  ///< update idx-th _bxs to idx-th leaf

    using namespace muda;
    ParallelFor().apply(
        size,
        [size, _primIdx = _primIdx, _primBox = _primBox, _primMap = _primMap, box = box] __device__(int idx)
        {
            int  newIdx      = _primMap[idx];
            aabb bv          = box[idx];
            _primIdx[newIdx] = idx;
            _primBox[newIdx] = bv;
        });
}


MUDA_INLINE void calcExtNodeSplitMetrics(int extsize, const uint32_t* _codes, int* _metrics)
{
    using namespace muda;
    ParallelFor().apply(
        extsize,
        [extsize = extsize, _codes = _codes, _metrics = _metrics] __device__(int idx)
        {
            _metrics[idx] =
                idx != extsize - 1 ? 32 - __clz(_codes[idx] ^ _codes[idx + 1]) : 33;
        });
}

MUDA_INLINE void buildIntNodes(int       size,
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
    using namespace muda;

    auto GridDim  = (size + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .apply(
            [=] __device__()
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
                    _tks_rc[cur] = idx, _tks_range_y[cur] = idx,
                    atomicOr(&_tks_mark[cur], 0x00000002), _lvs_mark[idx] = 0x00000007;
                else
                    _tks_lc[cur] = idx, _tks_range_x[cur] = idx,
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
                        _tks_box[cur].extend(_lvs_box[chr]);
                    }
                    else
                    {
                        _tks_box[cur].extend(_tks_box[chr]);
                    }

                    _tks_mark[cur] &= 0x00000007;
                    //if (idx < 10) printf("%d   %d\n", idx, _tks_mark[cur]);
                    //if (_tks_range_x[cur] == 0) printf("cur:%d  %d  %d   %d   %d\n", cur, _tks_range_x[252], _tks_range_y[252], _tks_lc[252], _tks_rc[252]);
                    l = _tks_range_x[cur] - 1, r = _tks_range_y[cur];
                    _lvs_lca[l + 1] = cur /*, _tks.rcd(cur) = ++_lvs.rcl(r)*/,
                                 _depths[l + 1]++;
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
                        atomicAnd(&_tks_mark[par], 0xFFFFFFFD),
                        _tks_mark[cur] |= 0x00000004;
                    else
                        _tks_lc[par] = cur, _tks_range_x[par] = l + 1,
                        atomicAnd(&_tks_mark[par], 0xFFFFFFFE),
                        _tks_mark[cur] &= 0xFFFFFFFB;
                    __threadfence();
                    cur = par;
                }
            });
}

__global__ void kbuildIntNodes(int       size,
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
            _tks_box[cur].extend(_lvs_box[chr]);
        }
        else
        {
            _tks_box[cur].extend(_tks_box[chr]);
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

MUDA_INLINE void calcIntNodeOrders(
    int size, int* _tks_lc, int* _lcas, uint32_t* _depths, uint32_t* _offsets, int* _tkMap)
{
    using namespace muda;

    ParallelFor().apply(size,
                        [=] __device__(int idx)
                        {
                            //for (; idx < size; idx += gridDim.x * blockDim.x) {
                            int node = _lcas[idx], depth = _depths[idx],
                                id = _offsets[idx];
                            //if (node == 874)printf("%d\n", idx);
                            if(node != -1)
                            {
                                for(; depth--; node = _tks_lc[node])
                                {
                                    _tkMap[node] = id++;
                                }
                            }
                            //}
                        });
}

MUDA_INLINE void updateBvhExtNodeLinks(int size, const int* _mapTable, int* _lcas, uint* _pars)
{
    using namespace muda;

    ParallelFor().apply(size,
                        [=] __device__(int idx)
                        {
                            int ori;
                            _pars[idx] = _mapTable[_pars[idx]];
                            if((ori = _lcas[idx]) != -1)
                                _lcas[idx] = _mapTable[ori] << 1;
                            else
                                _lcas[idx] = idx << 1 | 1;
                        });
}

MUDA_INLINE void setEscape(int* _lcas, int index)
{
    using namespace muda;

    Launch().apply([=] __device__() { _lcas[index] = -1; });
}

MUDA_INLINE void reorderNode(int            intSize,
                             const int*     _tkMap,
                             int*           _lvs_lca,
                             aabb*          _lvs_box,
                             int*           _unorderedTks_lc,
                             uint32_t*      _unorderedTks_mark,
                             int*           _unorderedTks_rangey,
                             aabb*          _unorderedTks_box,
                             stacklessnode* _nodes)
{
    using namespace muda;

    ParallelFor().apply(
        intSize + 1,
        [=] __device__(int idx)
        {
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
        });
}

MUDA_GENERIC MUDA_INLINE void quantilizeAABB(ulonglong2& qaabb, aabb& box, float3& origin, float3& delta)
{
    auto boxMin = box.min();
    auto boxMax = box.max();
    qaabb.x |= static_cast<ullint>((boxMin.x() - origin.x) / delta.x) << offset2;
    qaabb.x |= static_cast<ullint>((boxMin.y() - origin.y) / delta.y) << offset1;
    qaabb.x |= static_cast<ullint>((boxMin.z() - origin.z) / delta.z);

    qaabb.y |= static_cast<ullint>(ceilf((boxMax.x() - origin.x) / delta.x)) << offset2;
    qaabb.y |= static_cast<ullint>(ceilf((boxMax.y() - origin.y) / delta.y)) << offset1;
    qaabb.y |= static_cast<ullint>(ceilf((boxMax.z() - origin.z) / delta.z));
}

MUDA_INLINE void reorderQuantilizedNode(int         intSize,
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
    using namespace muda;

    ParallelFor().apply(
        intSize + 1,
        [=] __device__(int idx)
        {
            auto sceneMin = _scene_box[0].min();
            auto sceneMax = _scene_box[0].max();
            float3 origin = make_float3(sceneMin.x(), sceneMin.y(), sceneMin.z());
            float3 delta = make_float3(sceneMax.x(), sceneMax.y(), sceneMax.z()) - origin;

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

MUDA_GENERIC MUDA_INLINE int2 make_collision_pair(int a, int b)
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
MUDA_INLINE void quantilizedStacklessCDSharedSelf(Pred              pred,
                                                  uint              Size,
                                                  const aabb*       _box,
                                                  const int         intSize,
                                                  const int*        _lvs_idx,
                                                  const aabb*       scene,
                                                  const ulonglong2* _nodes,
                                                  int*              resCounter,
                                                  Vector2i*         res,
                                                  const int         maxRes)
{
    using namespace muda;

    auto numQuery = Size;
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .apply(
            [=] __device__()
            {
                int  tid      = blockIdx.x * blockDim.x + threadIdx.x;
                bool active   = tid < Size;
                auto sceneMin = scene[0].min();
                auto sceneMax = scene[0].max();
                float3 origin = make_float3(sceneMin.x(), sceneMin.y(), sceneMin.z());
                float3 delta =
                    make_float3(sceneMax.x(), sceneMax.y(), sceneMax.z()) - origin;
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

                            if(overlapsLonglong2int(node, bv)  // bounding volume overlap
                            )
                            {
                                if(lc == MaxIndex)
                                {
                                    if(tid < st - intSize)
                                    {
                                        auto pair =
                                            make_collision_pair(idx, _lvs_idx[st - intSize]);
                                        if(pred(pair.x, pair.y))  // user defined predicate
                                        {
                                            int sIdx = atomicAdd(&sharedCounter, 1);
                                            if(sIdx >= MAX_RES_PER_BLOCK)
                                            {
                                                break;
                                            }
                                            sharedRes[sIdx] = pair;
                                        }
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
                }
            });
}

template <typename Pred>
MUDA_INLINE void quantilizedStacklessCDSharedOther(Pred              pred,
                                                   uint              Size,
                                                   const aabb*       _box,
                                                   const int*        sortedIdx,
                                                   const int         intSize,
                                                   const int*        _lvs_idx,
                                                   const aabb*       scene,
                                                   const ulonglong2* _nodes,
                                                   int*              resCounter,
                                                   Vector2i*         res,
                                                   const int         maxRes)
{
    using namespace muda;

    auto numQuery = Size;
    auto GridDim  = (numQuery + 255) / 256;
    auto BlockDim = 256;

    Launch(GridDim, BlockDim)
        .apply(
            [=] __device__()
            {
                int  tid      = blockIdx.x * blockDim.x + threadIdx.x;
                bool active   = tid < Size;
                auto sceneMin = scene[0].min();
                auto sceneMax = scene[0].max();
                float3 origin = make_float3(sceneMin.x(), sceneMin.y(), sceneMin.z());
                float3 delta =
                    make_float3(sceneMax.x(), sceneMax.y(), sceneMax.z()) - origin;

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
                bool done = false;
                while(!done)
                {
                    __syncthreads();
                    if(active)
                    {
                        while(st != MaxIndex)
                        {
                            ulonglong2 node = _nodes[st];
                            lc              = node.x >> offset3;
                            escape          = node.y >> offset3;

                            if(overlapsLonglong2int(node, bv))  // bounding volume overlap
                            {
                                if(lc == MaxIndex)
                                {
                                    auto pair =
                                        make_collision_pair(idx, _lvs_idx[st - intSize]);
                                    if(pred(pair.x, pair.y))  // user defined predicate
                                    {
                                        int sIdx = atomicAdd(&sharedCounter, 1);
                                        if(sIdx >= MAX_RES_PER_BLOCK)
                                        {
                                            break;
                                        }
                                        sharedRes[sIdx] = pair;
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
                }
            });
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
                while(true)
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
                                            make_collision_pair(idx, _lvs_idx[st - intSize]);
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
                }
            });
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
                                        auto pair =
                                            make_collision_pair(idx, _lvs_idx[st - intSize]);
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
                }
            });
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

    if(m_config.quat_node)
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

    culbvh::calcMaxBVFromBox(
        numObjs, devicePtr, thrust::raw_pointer_cast(d_scene_box.data()));

    culbvh::calcMCsFromBox(numObjs,
                           devicePtr,
                           thrust::raw_pointer_cast(d_scene_box.data()),
                           thrust::raw_pointer_cast(d_mtcode.data()));

    thrust::sequence(thrust::device, d_sorted_id.begin(), d_sorted_id.end());
    thrust::sort_by_key(
        thrust::device, d_mtcode.begin(), d_mtcode.end(), d_sorted_id.begin());

    culbvh::calcInverseMapping(numObjs,
                               thrust::raw_pointer_cast(d_sorted_id.data()),
                               thrust::raw_pointer_cast(d_primMap.data()));

    culbvh::buildPrimitivesFromBox(numObjs,
                                   thrust::raw_pointer_cast(d_ext_idx.data()),
                                   thrust::raw_pointer_cast(d_ext_aabb.data()),
                                   thrust::raw_pointer_cast(d_primMap.data()),
                                   devicePtr);

    culbvh::calcExtNodeSplitMetrics(numObjs,
                                    thrust::raw_pointer_cast(d_mtcode.data()),
                                    thrust::raw_pointer_cast(d_metric.data()));

    culbvh::buildIntNodes(numObjs,
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

    //culbvh::kbuildIntNodes<<<(numObjs + 255) / 256, 256>>>(
    //    numObjs,
    //    thrust::raw_pointer_cast(d_count.data()),
    //    thrust::raw_pointer_cast(d_ext_lca.data()),
    //    thrust::raw_pointer_cast(d_metric.data()),
    //    thrust::raw_pointer_cast(d_ext_par.data()),
    //    thrust::raw_pointer_cast(d_ext_mark.data()),
    //    thrust::raw_pointer_cast(d_ext_aabb.data()),
    //    thrust::raw_pointer_cast(d_int_rc.data()),
    //    thrust::raw_pointer_cast(d_int_lc.data()),
    //    thrust::raw_pointer_cast(d_int_range_y.data()),
    //    thrust::raw_pointer_cast(d_int_range_x.data()),
    //    thrust::raw_pointer_cast(d_int_mark.data()),
    //    thrust::raw_pointer_cast(d_int_aabb.data()),
    //    thrust::raw_pointer_cast(d_flags.data()),
    //    thrust::raw_pointer_cast(d_int_par.data()));

    thrust::exclusive_scan(
        thrust::device, d_count.begin(), d_count.end(), d_offsetTable.begin());


    culbvh::calcIntNodeOrders(numObjs,
                              thrust::raw_pointer_cast(d_int_lc.data()),
                              thrust::raw_pointer_cast(d_ext_lca.data()),
                              thrust::raw_pointer_cast(d_count.data()),
                              thrust::raw_pointer_cast(d_offsetTable.data()),
                              thrust::raw_pointer_cast(d_tkMap.data()));

    thrust::fill(thrust::device, d_ext_lca.begin() + numObjs, d_ext_lca.begin() + numObjs + 1, -1);

    culbvh::updateBvhExtNodeLinks(numObjs,
                                  thrust::raw_pointer_cast(d_tkMap.data()),
                                  thrust::raw_pointer_cast(d_ext_lca.data()),
                                  thrust::raw_pointer_cast(d_ext_par.data()));

    if(m_config.quat_node)
    {

        culbvh::reorderQuantilizedNode(numInternalNodes,
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
        culbvh::reorderNode(numInternalNodes,
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

        if(m_config.quat_node)
        {
            culbvh::quantilizedStacklessCDSharedSelf(
                callback,
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

            culbvh::StacklessCDSharedSelf(callback,
                                          numQuery,
                                          thrust::raw_pointer_cast(d_objs),
                                          numObjs - 1,
                                          thrust::raw_pointer_cast(d_ext_idx.data()),
                                          thrust::raw_pointer_cast(d_nodes.data()),
                                          d_cpNum,
                                          d_cpRes,
                                          max_cpNum);
        }
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
    culbvh::calcMaxBVFromBox(numQuery, devicePtr, d_querySceneBox);

    culbvh::calcMCsFromBox(numQuery, devicePtr, d_querySceneBox, d_queryMtCode);

    thrust::sequence(thrust::device, d_querySortedId, d_querySortedId + numQuery);

    thrust::sort_by_key(thrust::device, d_queryMtCode, d_queryMtCode + numQuery, d_querySortedId);
}

template <std::invocable<IndexT, IndexT> Pred>
void StacklessBVH::query(muda::CBufferView<AABB> aabbs, Pred callback, QueryBuffer& qbuffer)
{
    using namespace muda;
    qbuffer.build(aabbs);

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

        if(m_config.quat_node)
        {
            culbvh::quantilizedStacklessCDSharedOther(
                callback,
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
        }
        else
        {
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
        }
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

    qbuffer.m_size = h_cp_num;
}
}  // namespace uipc::backend::cuda