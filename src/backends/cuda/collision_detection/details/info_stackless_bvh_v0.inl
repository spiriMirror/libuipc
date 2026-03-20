#include <cuda_device/builtin.h>
#include <muda/launch.h>

namespace uipc::info_stackless_v0_detail
{
using aabb = uipc::backend::cuda::AABB;
using node_t = uipc::backend::cuda::InfoStacklessBVHV0::Node;
using Vector2i = uipc::Vector2i;
using uint = uint32_t;
using ullint = unsigned long long;

constexpr int K_THREADS = 256;
constexpr int K_WARPS = K_THREADS >> 5;
constexpr int MAX_RES_PER_BLOCK = 1024;
constexpr int AABB_BITS = 15;
constexpr uint AABB_MASK = 0xFFFFFFFFu >> (32 - AABB_BITS);

struct PlainAABB
{
    float3 _min, _max;
};

MUDA_GENERIC MUDA_INLINE PlainAABB to_plain(const aabb& box)
{
    PlainAABB out;
    out._min = make_float3(box.min().x(), box.min().y(), box.min().z());
    out._max = make_float3(box.max().x(), box.max().y(), box.max().z());
    return out;
}

template <typename T>
MUDA_GENERIC MUDA_INLINE T mm_min(T a, T b)
{
    return a > b ? b : a;
}

template <typename T>
MUDA_GENERIC MUDA_INLINE T mm_max(T a, T b)
{
    return a > b ? a : b;
}

MUDA_DEVICE MUDA_INLINE float atomic_minf(float* addr, float value)
{
    return (value >= 0) ? __int_as_float(atomicMin((int*)addr, __float_as_int(value))) :
                          __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));
}

MUDA_DEVICE MUDA_INLINE float atomic_maxf(float* addr, float value)
{
    return (value >= 0) ? __int_as_float(atomicMax((int*)addr, __float_as_int(value))) :
                          __uint_as_float(atomicMin((unsigned int*)addr, __float_as_uint(value)));
}

MUDA_GENERIC MUDA_INLINE uint expand_bits(uint v)
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

MUDA_GENERIC MUDA_INLINE uint morton3D(float x, float y, float z)
{
    x = ::fmin(::fmax(x * 1024.0f, 0.0f), 1023.0f);
    y = ::fmin(::fmax(y * 1024.0f, 0.0f), 1023.0f);
    z = ::fmin(::fmax(z * 1024.0f, 0.0f), 1023.0f);
    uint xx = expand_bits((uint)x);
    uint yy = expand_bits((uint)y);
    uint zz = expand_bits((uint)z);
    return xx * 4 + yy * 2 + zz;
}

MUDA_GENERIC MUDA_INLINE Vector2i to_eigen(int2 v)
{
    return Vector2i{v.x, v.y};
}

MUDA_GENERIC MUDA_INLINE int2 ordered_pair(int a, int b)
{
    return (a < b) ? int2{a, b} : int2{b, a};
}

MUDA_GENERIC MUDA_INLINE float3 operator-(const float3& v0, const float3& v1)
{
    return make_float3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

MUDA_GENERIC MUDA_INLINE void safe_copy_to(int2* shared_res,
                                           int   total_in_block,
                                           Vector2i* global_res,
                                           int   global_idx,
                                           int   max_res)
{
    if(global_idx >= max_res || total_in_block == 0)
        return;
    auto copy_count = std::min(total_in_block, max_res - global_idx);
    int full_blocks = (copy_count - 1) / (int)blockDim.x;
    for(int i = 0; i < full_blocks; ++i)
    {
        int offset = i * blockDim.x + threadIdx.x;
        global_res[global_idx + offset] = to_eigen(shared_res[offset]);
    }
    int offset = full_blocks * blockDim.x + threadIdx.x;
    if(offset < copy_count)
        global_res[global_idx + offset] = to_eigen(shared_res[offset]);
}
}  // namespace uipc::info_stackless_v0_detail

namespace uipc::backend::cuda
{
using namespace info_stackless_v0_detail;

inline void InfoStacklessBVHV0::Impl::calcMaxBVFromBox(muda::CBufferView<AABB> aabbs,
                                                      muda::VarView<AABB> scene_box)
{
    if(aabbs.size() == 0)
        return;

    using namespace muda;
    auto num = aabbs.size();
    auto grid = (num + K_THREADS - 1) / K_THREADS;

    Launch(grid, K_THREADS).file_line(__FILE__, __LINE__).apply(
        [size = aabbs.size(), box = aabbs.viewer().name("box"), out = scene_box.viewer().name("out")] __device__()
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if(idx >= size)
                return;
            if(idx == 0)
                *out = AABB();

            __shared__ PlainAABB warp_boxes[K_WARPS];
            auto temp = to_plain(box(idx));
            int warp_tid = threadIdx.x & 31;
            int warp_id = threadIdx.x >> 5;

            float minx = temp._min.x, miny = temp._min.y, minz = temp._min.z;
            float maxx = temp._max.x, maxy = temp._max.y, maxz = temp._max.z;
            for(int i = 1; i < 32; i <<= 1)
            {
                minx = mm_min(minx, __shfl_down_sync(0xffffffff, minx, i));
                miny = mm_min(miny, __shfl_down_sync(0xffffffff, miny, i));
                minz = mm_min(minz, __shfl_down_sync(0xffffffff, minz, i));
                maxx = mm_max(maxx, __shfl_down_sync(0xffffffff, maxx, i));
                maxy = mm_max(maxy, __shfl_down_sync(0xffffffff, maxy, i));
                maxz = mm_max(maxz, __shfl_down_sync(0xffffffff, maxz, i));
            }
            if(warp_tid == 0)
            {
                warp_boxes[warp_id]._min = make_float3(minx, miny, minz);
                warp_boxes[warp_id]._max = make_float3(maxx, maxy, maxz);
            }
            __syncthreads();

            int warp_num = (blockIdx.x == gridDim.x - 1) ?
                               ((size - blockIdx.x * blockDim.x + 31) >> 5) :
                               (blockDim.x >> 5);
            if(threadIdx.x >= warp_num)
                return;

            temp = warp_boxes[threadIdx.x];
            minx = temp._min.x; miny = temp._min.y; minz = temp._min.z;
            maxx = temp._max.x; maxy = temp._max.y; maxz = temp._max.z;
            for(int i = 1; i < warp_num; i <<= 1)
            {
                minx = mm_min(minx, __shfl_down_sync(0xffffffff, minx, i));
                miny = mm_min(miny, __shfl_down_sync(0xffffffff, miny, i));
                minz = mm_min(minz, __shfl_down_sync(0xffffffff, minz, i));
                maxx = mm_max(maxx, __shfl_down_sync(0xffffffff, maxx, i));
                maxy = mm_max(maxy, __shfl_down_sync(0xffffffff, maxy, i));
                maxz = mm_max(maxz, __shfl_down_sync(0xffffffff, maxz, i));
            }
            if(threadIdx.x == 0)
            {
                atomic_minf(&out->min().x(), minx);
                atomic_minf(&out->min().y(), miny);
                atomic_minf(&out->min().z(), minz);
                atomic_maxf(&out->max().x(), maxx);
                atomic_maxf(&out->max().y(), maxy);
                atomic_maxf(&out->max().z(), maxz);
            }
        });
}

inline void InfoStacklessBVHV0::Impl::calcMCsFromBox(muda::CBufferView<AABB> aabbs,
                                                    muda::CVarView<AABB> scene_box,
                                                    muda::BufferView<uint32_t> codes)
{
    using namespace muda;
    ParallelFor().file_line(__FILE__, __LINE__).apply(
        aabbs.size(),
        [box = aabbs.viewer().name("box"), scene = scene_box.viewer().name("scene"), codes = codes.viewer().name("codes")] __device__(int idx)
        {
            auto bv = box(idx);
            auto center = bv.center();
            float3 c = make_float3(center.x(), center.y(), center.z());
            auto scene_min = scene->min();
            float3 smin = make_float3(scene_min.x(), scene_min.y(), scene_min.z());
            auto scene_size = scene->sizes();
            float3 off = c - smin;
            codes(idx) = morton3D(off.x / scene_size.x(), off.y / scene_size.y(), off.z / scene_size.z());
        });
}

inline void InfoStacklessBVHV0::Impl::calcInverseMapping()
{
    using namespace muda;
    ParallelFor().file_line(__FILE__, __LINE__).apply(
        sorted_id.size(),
        [map = sorted_id.viewer().name("map"), inv = primMap.viewer().name("inv")] __device__(int idx)
        { inv(map(idx)) = idx; });
}

inline void InfoStacklessBVHV0::Impl::buildPrimitivesFromBox(muda::CBufferView<AABB> aabbs)
{
    using namespace muda;
    constexpr IndexT invalid = static_cast<IndexT>(-1);
    bool has_info = bids.size() == aabbs.size() && cids.size() == aabbs.size();
    ParallelFor().apply(
        aabbs.size(),
        [_prim_idx = ext_idx.viewer().name("prim_idx"),
         _prim_box = ext_aabb.viewer().name("prim_box"),
         _prim_map = primMap.viewer().name("prim_map"),
         _ext_bid = ext_bid.viewer().name("ext_bid"),
         _ext_cid = ext_cid.viewer().name("ext_cid"),
         _bids = bids.viewer().name("bids"),
         _cids = cids.viewer().name("cids"),
         has_info,
         box = aabbs.viewer().name("box")] __device__(int idx)
        {
            int new_idx = _prim_map(idx);
            _prim_idx(new_idx) = idx;
            _prim_box(new_idx) = box(idx);
            if(has_info)
            {
                _ext_bid(new_idx) = _bids(idx);
                _ext_cid(new_idx) = _cids(idx);
            }
            else
            {
                _ext_bid(new_idx) = invalid;
                _ext_cid(new_idx) = invalid;
            }
        });
}

inline void InfoStacklessBVHV0::Impl::calcExtNodeSplitMetrics()
{
    using namespace muda;
    ParallelFor().file_line(__FILE__, __LINE__).apply(
        mtcode.size(),
        [n = mtcode.size(), codes = mtcode.viewer().name("codes"), metrics = metric.viewer().name("metrics")] __device__(int idx)
        { metrics(idx) = idx != n - 1 ? 32 - __clz(codes(idx) ^ codes(idx + 1)) : 33; });
}

inline void InfoStacklessBVHV0::Impl::buildIntNodes(int size)
{
    using namespace muda;
    constexpr IndexT invalid = static_cast<IndexT>(-1);
    auto grid = (size + 255) / 256;
    Launch(grid, 256).file_line(__FILE__, __LINE__).apply(
        [size,
         _depths = count.viewer().name("depths"),
         _lvs_lca = ext_lca.viewer().name("lvs_lca"),
         _lvs_metric = metric.viewer().name("lvs_metric"),
         _lvs_par = ext_par.viewer().name("lvs_par"),
         _lvs_box = ext_aabb.viewer().name("lvs_box"),
         _lvs_bid = ext_bid.viewer().name("lvs_bid"),
         _lvs_cid = ext_cid.viewer().name("lvs_cid"),
         _tks_lc = int_lc.viewer().name("tks_lc"),
         _tks_rc = int_rc.viewer().name("tks_rc"),
         _tks_range_x = int_range_x.viewer().name("tks_range_x"),
         _tks_range_y = int_range_y.viewer().name("tks_range_y"),
         _tks_mark = int_mark.viewer().name("tks_mark"),
         _tks_box = int_aabb.viewer().name("tks_box"),
         _tks_bid = int_bid.viewer().name("tks_bid"),
         _tks_cid = int_cid.viewer().name("tks_cid"),
         _flag = flags.viewer().name("flag"),
         _tks_par = int_par.viewer().name("tks_par")] __device__()
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if(idx >= size)
                return;

            _lvs_lca(idx) = -1;
            _depths(idx) = 0;
            int l = idx - 1;
            int r = idx;
            bool mark = (l >= 0) ? (_lvs_metric(l) < _lvs_metric(r)) : false;
            int cur = mark ? l : r;
            _lvs_par(idx) = cur;
            if(_flag.total_size() == 0)
                return;

            if(mark)
            {
                _tks_rc(cur) = idx;
                _tks_range_y(cur) = idx;
                atomicOr(&_tks_mark(cur), 0x00000002);
            }
            else
            {
                _tks_lc(cur) = idx;
                _tks_range_x(cur) = idx;
                atomicOr(&_tks_mark(cur), 0x00000001);
            }
            __threadfence();

            while(atomicAdd(&_flag(cur), 1) == 1)
            {
                int chl = _tks_lc(cur);
                int chr = _tks_rc(cur);
                uint32_t m = _tks_mark(cur);
                if(m & 1) _tks_box(cur) = _lvs_box(chl);
                else _tks_box(cur) = _tks_box(chl);
                if(m & 2) _tks_box(cur).extend(_lvs_box(chr));
                else _tks_box(cur).extend(_tks_box(chr));

                IndexT l_bid = (m & 1) ? _lvs_bid(chl) : _tks_bid(chl);
                IndexT r_bid = (m & 2) ? _lvs_bid(chr) : _tks_bid(chr);
                IndexT l_cid = (m & 1) ? _lvs_cid(chl) : _tks_cid(chl);
                IndexT r_cid = (m & 2) ? _lvs_cid(chr) : _tks_cid(chr);
                _tks_bid(cur) = (l_bid == r_bid) ? l_bid : invalid;
                _tks_cid(cur) = (l_cid == r_cid) ? l_cid : invalid;

                _tks_mark(cur) &= 0x00000007;
                l = _tks_range_x(cur) - 1;
                r = _tks_range_y(cur);
                _lvs_lca(l + 1) = cur;
                _depths(l + 1)++;
                mark = (l >= 0) ? (_lvs_metric(l) < _lvs_metric(r)) : false;
                if(l + 1 == 0 && r == size - 1)
                {
                    _tks_par(cur) = -1;
                    _tks_mark(cur) &= 0xFFFFFFFB;
                    break;
                }

                int par = mark ? l : r;
                _tks_par(cur) = par;
                if(mark)
                {
                    _tks_rc(par) = cur;
                    _tks_range_y(par) = r;
                    atomicAnd(&_tks_mark(par), 0xFFFFFFFD);
                    _tks_mark(cur) |= 0x00000004;
                }
                else
                {
                    _tks_lc(par) = cur;
                    _tks_range_x(par) = l + 1;
                    atomicAnd(&_tks_mark(par), 0xFFFFFFFE);
                    _tks_mark(cur) &= 0xFFFFFFFB;
                }
                __threadfence();
                cur = par;
            }
        });
}

inline void InfoStacklessBVHV0::Impl::calcIntNodeOrders(int size)
{
    using namespace muda;
    ParallelFor().file_line(__FILE__, __LINE__).apply(
        size,
        [_tks_lc = int_lc.viewer().name("tks_lc"),
         _lcas = ext_lca.viewer().name("lcas"),
         _depths = count.viewer().name("depths"),
         _offsets = offsetTable.viewer().name("offsets"),
         _tkMap = tkMap.viewer().name("tkMap")] __device__(int idx)
        {
            int node = _lcas(idx);
            int depth = _depths(idx);
            int id = _offsets(idx);
            if(node != -1)
            {
                for(; depth--; node = _tks_lc(node))
                    _tkMap(node) = id++;
            }
        });
}

inline void InfoStacklessBVHV0::Impl::updateBvhExtNodeLinks(int size)
{
    using namespace muda;
    if(flags.size() == 0)
        return;
    ParallelFor().file_line(__FILE__, __LINE__).apply(
        size,
        [_map = tkMap.viewer().name("map"), _lcas = ext_lca.viewer().name("lcas"), _pars = ext_par.viewer().name("pars")] __device__(int idx)
        {
            _pars(idx) = _map(_pars(idx));
            int ori = _lcas(idx);
            _lcas(idx) = (ori != -1) ? (_map(ori) << 1) : (idx << 1 | 1);
        });
}

inline void InfoStacklessBVHV0::Impl::reorderNode(int int_size)
{
    using namespace muda;
    constexpr IndexT invalid = static_cast<IndexT>(-1);
    ParallelFor().file_line(__FILE__, __LINE__).apply(
        int_size + 1,
        [int_size,
         _lvs_lca = ext_lca.viewer().name("lvs_lca"),
         _lvs_box = ext_aabb.viewer().name("lvs_box"),
         _lvs_bid = ext_bid.viewer().name("lvs_bid"),
         _lvs_cid = ext_cid.viewer().name("lvs_cid"),
         _tk_map = tkMap.viewer().name("tk_map"),
         _int_lc = int_lc.viewer().name("int_lc"),
         _int_mark = int_mark.viewer().name("int_mark"),
         _int_range_y = int_range_y.viewer().name("int_range_y"),
         _int_box = int_aabb.viewer().name("int_box"),
         _int_bid = int_bid.viewer().name("int_bid"),
         _int_cid = int_cid.viewer().name("int_cid"),
         _nodes = nodes.viewer().name("nodes")] __device__(int idx)
        {
            Node leaf;
            leaf.lc = -1;
            int escape = _lvs_lca(idx + 1);
            if(escape == -1) leaf.escape = -1;
            else
            {
                int b_leaf = escape & 1;
                escape >>= 1;
                leaf.escape = escape + (b_leaf ? int_size : 0);
            }
            leaf.bound = _lvs_box(idx);
            leaf.bid = _lvs_bid(idx);
            leaf.cid = _lvs_cid(idx);
            _nodes(idx + int_size) = leaf;

            if(idx >= int_size)
                return;

            Node n;
            int new_id = _tk_map(idx);
            uint32_t m = _int_mark(idx);
            n.lc = (m & 1) ? _int_lc(idx) + int_size : _tk_map(_int_lc(idx));
            n.bound = _int_box(idx);
            int ie = _lvs_lca(_int_range_y(idx) + 1);
            if(ie == -1) n.escape = -1;
            else
            {
                int b_leaf = ie & 1;
                ie >>= 1;
                n.escape = ie + (b_leaf ? int_size : 0);
            }
            n.bid = _int_bid(idx);
            n.cid = _int_cid(idx);
            _nodes(new_id) = n;
        });
}

inline void InfoStacklessBVHV0::Impl::propagateInformativeMetadata(int)
{
}

inline void InfoStacklessBVHV0::Impl::build(muda::CBufferView<AABB>   aabbs,
                                          muda::CBufferView<IndexT> _bids,
                                          muda::CBufferView<IndexT> _cids)
{
    objs = aabbs;
    bids = _bids;
    cids = _cids;
    auto num_objs = aabbs.size();
    if(num_objs == 0)
        return;

    auto num_internal = num_objs - 1;
    auto num_nodes = num_objs * 2 - 1;
    mtcode.resize(num_objs);
    sorted_id.resize(num_objs);
    primMap.resize(num_objs);
    ext_aabb.resize(num_objs);
    ext_idx.resize(num_objs);
    ext_lca.resize(num_objs + 1);
    ext_par.resize(num_objs);
    ext_mark.resize(num_objs);
    ext_bid.resize(num_objs);
    ext_cid.resize(num_objs);
    metric.resize(num_objs);
    tkMap.resize(num_objs);
    offsetTable.resize(num_objs);
    count.resize(num_objs);
    flags.resize(num_internal);
    int_lc.resize(num_internal);
    int_rc.resize(num_internal);
    int_par.resize(num_internal);
    int_range_x.resize(num_internal);
    int_range_y.resize(num_internal);
    int_mark.resize(num_internal);
    int_aabb.resize(num_internal);
    int_bid.resize(num_internal);
    int_cid.resize(num_internal);
    nodes.resize(num_nodes);

    thrust::fill(flags.begin(), flags.end(), 0);
    thrust::fill(thrust::device, ext_mark.begin(), ext_mark.end(), 7);
    thrust::fill(thrust::device, ext_lca.begin(), ext_lca.end(), 0);
    thrust::fill(thrust::device, ext_par.begin(), ext_par.end(), 0);
    thrust::fill(thrust::device, int_bid.begin(), int_bid.end(), static_cast<IndexT>(-1));
    thrust::fill(thrust::device, int_cid.begin(), int_cid.end(), static_cast<IndexT>(-1));

    calcMaxBVFromBox(aabbs, scene_box.view());
    calcMCsFromBox(aabbs, scene_box.view(), mtcode.view());
    auto null_stream = thrust::cuda::par_nosync.on(nullptr);
    thrust::sequence(null_stream, sorted_id.begin(), sorted_id.end());
    thrust::sort_by_key(null_stream, mtcode.begin(), mtcode.end(), sorted_id.begin());
    calcInverseMapping();
    buildPrimitivesFromBox(aabbs);
    calcExtNodeSplitMetrics();
    buildIntNodes(num_objs);
    thrust::exclusive_scan(null_stream, count.begin(), count.end(), offsetTable.begin());
    calcIntNodeOrders(num_objs);
    thrust::fill(null_stream, ext_lca.begin() + num_objs, ext_lca.begin() + num_objs + 1, -1);
    updateBvhExtNodeLinks(num_objs);
    reorderNode(num_internal);
}

template <typename NodeCull, typename PairPred>
void InfoStacklessBVHV0::Impl::stacklessSelf(NodeCull node_cull,
                                           PairPred pair_pred,
                                           muda::VarView<int> cpNum,
                                           muda::BufferView<Vector2i> buffer)
{
    using namespace muda;
    auto num_query = static_cast<int>(ext_aabb.size());
    auto num_objs = num_query;
    auto grid = (num_query + K_THREADS - 1) / K_THREADS;
    Launch(grid, K_THREADS).apply(
        [Size = num_query,
         _box = objs.viewer().name("box"),
         intSize = num_objs - 1,
         numObjs = num_objs,
         _lvs_idx = ext_idx.viewer().name("lvs_idx"),
         _nodes = nodes.viewer().name("nodes"),
         resCounter = cpNum.viewer().name("cp_num"),
         res = buffer.viewer().name("res"),
         node_cull,
         pair_pred] __device__()
        {
            int tid = blockIdx.x * blockDim.x + threadIdx.x;
            bool active = tid < Size;
            int idx = -1;
            AABB bv;
            if(active)
            {
                idx = _lvs_idx(tid);
                bv = _box(idx);
            }

            __shared__ int2 shared_res[MAX_RES_PER_BLOCK];
            __shared__ int shared_counter;
            __shared__ int shared_global_idx;
            if(threadIdx.x == 0)
                shared_counter = 0;

            int st = 0;
            const int max_iter = numObjs * 2;
            while(true)
            {
                __syncthreads();
                if(active)
                {
                    int inner_i = 0;
                    for(; inner_i < max_iter; ++inner_i)
                    {
                        if(st == -1)
                            break;
                        auto node = _nodes(st);
                        if(!node.bound.intersects(bv))
                        {
                            st = node.escape;
                            continue;
                        }
                        if(!node_cull(idx, node.bid, node.cid))
                        {
                            st = node.escape;
                            continue;
                        }
                        if(node.lc == -1)
                        {
                            if(tid < st - intSize)
                            {
                                auto pair = ordered_pair(idx, _lvs_idx(st - intSize));
                                if(pair_pred(pair.x, pair.y))
                                {
                                    int sidx = atomicAdd(&shared_counter, 1);
                                    if(sidx >= MAX_RES_PER_BLOCK)
                                        break;
                                    shared_res[sidx] = pair;
                                }
                            }
                            st = node.escape;
                        }
                        else
                            st = node.lc;
                    }
                    MUDA_ASSERT(inner_i < max_iter, "Exceeded max stackless iteration");
                }
                __syncthreads();
                int total = min(shared_counter, MAX_RES_PER_BLOCK);
                if(threadIdx.x == 0)
                    shared_global_idx = atomicAdd(resCounter.data(), total);
                __syncthreads();
                int gidx = shared_global_idx;
                if(threadIdx.x == 0)
                    shared_counter = 0;
                bool done = total < MAX_RES_PER_BLOCK;
                safe_copy_to(shared_res, total, res.data(), gidx, static_cast<int>(res.total_size()));
                if(done)
                    break;
            }
        });
}

template <typename NodeCull, typename PairPred>
void InfoStacklessBVHV0::Impl::stacklessOther(NodeCull node_cull,
                                            PairPred pair_pred,
                                            muda::CBufferView<AABB> query_aabbs,
                                            muda::CBufferView<int> query_sorted_id,
                                            muda::VarView<int> cpNum,
                                            muda::BufferView<Vector2i> buffer)
{
    using namespace muda;
    auto num_query = static_cast<int>(query_aabbs.size());
    auto num_objs = static_cast<int>(ext_aabb.size());
    auto grid = (num_query + K_THREADS - 1) / K_THREADS;
    Launch(grid, K_THREADS).apply(
        [Size = num_query,
         _box = query_aabbs.viewer().name("qbox"),
         sortedIdx = query_sorted_id.viewer().name("sortedIdx"),
         intSize = num_objs - 1,
         numObjs = num_objs,
         _lvs_idx = ext_idx.viewer().name("lvs_idx"),
         _nodes = nodes.viewer().name("nodes"),
         resCounter = cpNum.viewer().name("cp_num"),
         res = buffer.viewer().name("res"),
         node_cull,
         pair_pred] __device__()
        {
            int tid = blockIdx.x * blockDim.x + threadIdx.x;
            bool active = tid < Size;
            int idx = -1;
            AABB bv;
            if(active)
            {
                idx = sortedIdx(tid);
                bv = _box(idx);
            }

            __shared__ int2 shared_res[MAX_RES_PER_BLOCK];
            __shared__ int shared_counter;
            __shared__ int shared_global_idx;
            if(threadIdx.x == 0)
                shared_counter = 0;

            int st = 0;
            const int max_iter = numObjs * 2;
            while(true)
            {
                __syncthreads();
                if(active)
                {
                    int inner_i = 0;
                    for(; inner_i < max_iter; ++inner_i)
                    {
                        if(st == -1)
                            break;
                        auto node = _nodes(st);
                        if(!node.bound.intersects(bv))
                        {
                            st = node.escape;
                            continue;
                        }
                        if(!node_cull(idx, node.bid, node.cid))
                        {
                            st = node.escape;
                            continue;
                        }
                        if(node.lc == -1)
                        {
                            auto pair = int2{idx, _lvs_idx(st - intSize)};
                            if(pair_pred(pair.x, pair.y))
                            {
                                int sidx = atomicAdd(&shared_counter, 1);
                                if(sidx >= MAX_RES_PER_BLOCK)
                                    break;
                                shared_res[sidx] = pair;
                            }
                            st = node.escape;
                        }
                        else
                            st = node.lc;
                    }
                    MUDA_ASSERT(inner_i < max_iter, "Exceeded max stackless iteration");
                }

                __syncthreads();
                int total = min(shared_counter, MAX_RES_PER_BLOCK);
                if(threadIdx.x == 0)
                    shared_global_idx = atomicAdd(resCounter.data(), total);
                __syncthreads();
                int gidx = shared_global_idx;
                if(threadIdx.x == 0)
                    shared_counter = 0;
                __syncthreads();
                bool done = total < MAX_RES_PER_BLOCK;
                safe_copy_to(shared_res, total, res.data(), gidx, static_cast<int>(res.total_size()));
                if(done)
                    break;
            }
        });
}

inline InfoStacklessBVHV0::InfoStacklessBVHV0(muda::Stream& stream) noexcept
{
    (void)stream;
}

inline void InfoStacklessBVHV0::QueryBuffer::build(muda::CBufferView<AABB> aabbs)
{
    m_queryMtCode.resize(aabbs.size());
    m_querySortedId.resize(aabbs.size());
    Impl::calcMaxBVFromBox(aabbs, m_querySceneBox);
    Impl::calcMCsFromBox(aabbs, m_querySceneBox, m_queryMtCode.view());
    auto null_stream = thrust::cuda::par_nosync.on(nullptr);
    auto n = static_cast<int>(aabbs.size());
    auto d_codes = m_queryMtCode.data();
    auto d_ids = m_querySortedId.data();
    thrust::sequence(null_stream, d_ids, d_ids + n);
    thrust::sort_by_key(null_stream, d_codes, d_codes + n, d_ids);
}

inline void InfoStacklessBVHV0::build(muda::CBufferView<AABB> aabbs,
                                    muda::CBufferView<IndexT> BIDs,
                                    muda::CBufferView<IndexT> CIDs)
{
    m_aabbs = aabbs;
    m_BIDs = BIDs;
    m_CIDs = CIDs;
    UIPC_ASSERT(m_aabbs.size() == m_BIDs.size(), "AABB and BID size mismatch. aabbs=%zu, bids=%zu", m_aabbs.size(), m_BIDs.size());
    UIPC_ASSERT(m_aabbs.size() == m_CIDs.size(), "AABB and CID size mismatch. aabbs=%zu, cids=%zu", m_aabbs.size(), m_CIDs.size());
    m_impl.build(aabbs, BIDs, CIDs);
}

inline void InfoStacklessBVHV0::build(muda::CBufferView<AABB> aabbs)
{
    m_aabbs = aabbs;
    m_BIDs = {};
    m_CIDs = {};
    m_impl.build(aabbs, {}, {});
}

template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVHV0::detect(muda::CBuffer2DView<IndexT> cmts, NodePred np, LeafPred lp, QueryBuffer& qbuffer)
{
    if(m_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }
    UIPC_ASSERT(m_aabbs.size() == m_BIDs.size(), "AABB and BID size mismatch. aabbs=%zu, bids=%zu", m_aabbs.size(), m_BIDs.size());
    UIPC_ASSERT(m_aabbs.size() == m_CIDs.size(), "AABB and CID size mismatch. aabbs=%zu, cids=%zu", m_aabbs.size(), m_CIDs.size());

    using namespace muda;
    constexpr IndexT invalid = static_cast<IndexT>(-1);
    auto do_query = [&]
    {
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);
        m_impl.stacklessSelf(
            [bids = m_BIDs.viewer().name("bids"), cids = m_CIDs.viewer().name("cids"), cmts = cmts.viewer().name("cmts"), np = np] __device__(IndexT i, IndexT node_bid, IndexT node_cid)
            {
                NodePredInfo info{i, node_bid, node_cid};
                return np(info);
            },
            [lp = lp] __device__(IndexT i, IndexT j)
            {
                if(j <= i)
                    return false;
                return lp(i, j);
            },
            qbuffer.m_cpNum.view(),
            qbuffer.m_pairs.view());
    };

    do_query();
    int h_cp_num = qbuffer.m_cpNum;
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_impl.config.reserve_ratio);
        do_query();
    }
    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}

template <std::invocable<IndexT, IndexT> Pred>
inline void InfoStacklessBVHV0::detect(Pred callback, QueryBuffer& qbuffer)
{
    if(m_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }
    using namespace muda;
    auto do_query = [&]
    {
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);
        m_impl.stacklessSelf(
            [] __device__(IndexT, IndexT, IndexT) { return true; },
            callback,
            qbuffer.m_cpNum.view(),
            qbuffer.m_pairs.view());
    };
    do_query();
    int h_cp_num = qbuffer.m_cpNum;
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_impl.config.reserve_ratio);
        do_query();
    }
    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}

template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVHV0::query(muda::CBufferView<AABB> query_aabbs,
                                    muda::CBufferView<IndexT> query_BIDs,
                                    muda::CBufferView<IndexT> query_CIDs,
                                    muda::CBuffer2DView<IndexT> cmts,
                                    NodePred np,
                                    LeafPred lp,
                                    QueryBuffer& qbuffer)
{
    if(m_aabbs.size() == 0 || query_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }
    UIPC_ASSERT(query_aabbs.size() == query_BIDs.size(), "Query AABB and BID size mismatch. aabbs=%zu, bids=%zu", query_aabbs.size(), query_BIDs.size());
    UIPC_ASSERT(query_aabbs.size() == query_CIDs.size(), "Query AABB and CID size mismatch. aabbs=%zu, cids=%zu", query_aabbs.size(), query_CIDs.size());

    using namespace muda;
    constexpr IndexT invalid = static_cast<IndexT>(-1);
    qbuffer.build(query_aabbs);
    auto do_query = [&]
    {
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);
        m_impl.stacklessOther(
            [np = np] __device__(IndexT i, IndexT node_bid, IndexT node_cid)
            {
                NodePredInfo info{i, node_bid, node_cid};
                return np(info);
            },
            [lp = lp] __device__(IndexT i, IndexT j)
            {
                return lp(i, j);
            },
            query_aabbs,
            qbuffer.m_querySortedId.view(),
            qbuffer.m_cpNum.view(),
            qbuffer.m_pairs.view());
    };
    do_query();
    int h_cp_num = qbuffer.m_cpNum;
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_impl.config.reserve_ratio);
        do_query();
    }
    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}

template <std::invocable<IndexT, IndexT> Pred>
inline void InfoStacklessBVHV0::query(muda::CBufferView<AABB> query_aabbs,
                                    Pred callback,
                                    QueryBuffer& qbuffer)
{
    if(m_aabbs.size() == 0 || query_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }
    using namespace muda;
    qbuffer.build(query_aabbs);
    auto do_query = [&]
    {
        BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);
        m_impl.stacklessOther(
            [] __device__(IndexT, IndexT, IndexT) { return true; },
            callback,
            query_aabbs,
            qbuffer.m_querySortedId.view(),
            qbuffer.m_cpNum.view(),
            qbuffer.m_pairs.view());
    };
    do_query();
    int h_cp_num = qbuffer.m_cpNum;
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_impl.config.reserve_ratio);
        do_query();
    }
    UIPC_ASSERT(h_cp_num >= 0, "fatal error");
    qbuffer.m_size = h_cp_num;
}
}  // namespace uipc::backend::cuda
