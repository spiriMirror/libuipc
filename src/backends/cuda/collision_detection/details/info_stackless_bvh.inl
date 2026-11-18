/*
 * NVIDIA contributions: scene-bounds reduction fixes, duplicate-half
 * self-query pruning and trajectory refit/query reuse; rank-ordered
 * Morton-code ties in the build, build, refit and query launches on a given
 * stream, 128-thread refit blocks, and the warp-cooperative Other-query
 * traversal.
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

#include <cuda_device/builtin.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

// Optimized InfoStacklessBVH implementation. Self/Other traversal pre-loads
// per-query bid/cid into shared memory, and Self also skips subtrees that
// cannot pass the Morton-rank uniqueness rule. The build path keeps enough
// topology metadata for later refits, while compatible Other queries may
// reuse their Morton order; both optimizations fall back to a full build when
// their cached topology or ordering is no longer valid.

namespace uipc::info_stackless_detail
{
using aabb     = uipc::backend::cuda::AABB;
using node_t   = uipc::backend::cuda::InfoStacklessBVH::Node;
using Vector2i = uipc::Vector2i;
using uint     = uint32_t;
using ullint   = unsigned long long;

#ifndef UIPC_INFO_STACKLESS_BVH_SELF_THREADS
#define UIPC_INFO_STACKLESS_BVH_SELF_THREADS 64
#endif
#ifndef UIPC_INFO_STACKLESS_BVH_SELF_QUEUE_SLOTS_PER_THREAD
#define UIPC_INFO_STACKLESS_BVH_SELF_QUEUE_SLOTS_PER_THREAD 4
#endif
#ifndef UIPC_INFO_STACKLESS_BVH_OTHER_THREADS
#define UIPC_INFO_STACKLESS_BVH_OTHER_THREADS 128
#endif
#ifndef UIPC_INFO_STACKLESS_BVH_OTHER_QUEUE_SLOTS_PER_THREAD
#define UIPC_INFO_STACKLESS_BVH_OTHER_QUEUE_SLOTS_PER_THREAD 4
#endif

constexpr int K_BUILD_THREADS = 256;
constexpr int K_BUILD_WARPS   = K_BUILD_THREADS >> 5;

constexpr int K_SELF_THREADS = UIPC_INFO_STACKLESS_BVH_SELF_THREADS;
constexpr int K_SELF_QUEUE_SLOTS_PER_THREAD = UIPC_INFO_STACKLESS_BVH_SELF_QUEUE_SLOTS_PER_THREAD;
constexpr int K_SELF_MAX_RES_PER_BLOCK = K_SELF_THREADS * K_SELF_QUEUE_SLOTS_PER_THREAD;

constexpr int K_OTHER_THREADS = UIPC_INFO_STACKLESS_BVH_OTHER_THREADS;
constexpr int K_OTHER_QUEUE_SLOTS_PER_THREAD =
    UIPC_INFO_STACKLESS_BVH_OTHER_QUEUE_SLOTS_PER_THREAD;
constexpr int K_OTHER_MAX_RES_PER_BLOCK = K_OTHER_THREADS * K_OTHER_QUEUE_SLOTS_PER_THREAD;
// Per-warp leaf queue (two warps of entries) and task pool of the
// warp-cooperative Other traversal.
constexpr int K_OTHER_WARP_QUEUE = 64;
constexpr int K_OTHER_WARP_TASKS = 64;

static_assert(K_BUILD_THREADS % 32 == 0);
static_assert(K_SELF_THREADS > 0 && K_SELF_THREADS <= 1024 && K_SELF_THREADS % 32 == 0);
static_assert(K_OTHER_THREADS > 0 && K_OTHER_THREADS <= 1024 && K_OTHER_THREADS % 32 == 0);
static_assert(K_SELF_QUEUE_SLOTS_PER_THREAD > 0);
static_assert(K_OTHER_QUEUE_SLOTS_PER_THREAD > 0);
constexpr int  AABB_BITS = 15;
constexpr uint AABB_MASK = 0xFFFFFFFFu >> (32 - AABB_BITS);

struct PlainAABB
{
    float3 _min, _max;
};

UIPC_GENERIC UIPC_INLINE PlainAABB to_plain(const aabb& box)
{
    PlainAABB out;
    out._min = make_float3(box.min().x(), box.min().y(), box.min().z());
    out._max = make_float3(box.max().x(), box.max().y(), box.max().z());
    return out;
}

template <typename T>
UIPC_GENERIC UIPC_INLINE T mm_min(T a, T b)
{
    return a > b ? b : a;
}

template <typename T>
UIPC_GENERIC UIPC_INLINE T mm_max(T a, T b)
{
    return a > b ? a : b;
}

UIPC_DEVICE UIPC_INLINE float atomic_minf(float* addr, float value)
{
    // Classify by the IEEE-754 sign bit: -0.0f compares >= 0.0f but must use
    // the negative-value integer ordering.
    const int value_bits = __float_as_int(value);
    return (value_bits >= 0) ?
               __int_as_float(atomicMin((int*)addr, value_bits)) :
               __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));
}

UIPC_DEVICE UIPC_INLINE float atomic_maxf(float* addr, float value)
{
    // See atomic_minf: a numeric comparison misclassifies negative zero.
    const int value_bits = __float_as_int(value);
    return (value_bits >= 0) ?
               __int_as_float(atomicMax((int*)addr, value_bits)) :
               __uint_as_float(atomicMin((unsigned int*)addr, __float_as_uint(value)));
}

UIPC_GENERIC UIPC_INLINE uint expand_bits(uint v)
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

UIPC_GENERIC UIPC_INLINE uint morton3D(float x, float y, float z)
{
    x       = ::fmin(::fmax(x * 1024.0f, 0.0f), 1023.0f);
    y       = ::fmin(::fmax(y * 1024.0f, 0.0f), 1023.0f);
    z       = ::fmin(::fmax(z * 1024.0f, 0.0f), 1023.0f);
    uint xx = expand_bits((uint)x);
    uint yy = expand_bits((uint)y);
    uint zz = expand_bits((uint)z);
    return xx * 4 + yy * 2 + zz;
}

UIPC_GENERIC UIPC_INLINE Vector2i to_eigen(int2 v)
{
    return Vector2i{v.x, v.y};
}

UIPC_GENERIC UIPC_INLINE int2 ordered_pair(int a, int b)
{
    return (a < b) ? int2{a, b} : int2{b, a};
}

UIPC_GENERIC UIPC_INLINE float3 operator-(const float3& v0, const float3& v1)
{
    return make_float3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

UIPC_GENERIC UIPC_INLINE void safe_copy_to(int2*     shared_res,
                                           int       total_in_block,
                                           Vector2i* global_res,
                                           int       global_idx,
                                           int       max_res)
{
    if(global_idx >= max_res || total_in_block == 0)
        return;
    auto copy_count  = std::min(total_in_block, max_res - global_idx);
    int  full_blocks = (copy_count - 1) / (int)blockDim.x;
    for(int i = 0; i < full_blocks; ++i)
    {
        int offset                      = i * blockDim.x + threadIdx.x;
        global_res[global_idx + offset] = to_eigen(shared_res[offset]);
    }
    int offset = full_blocks * blockDim.x + threadIdx.x;
    if(offset < copy_count)
        global_res[global_idx + offset] = to_eigen(shared_res[offset]);
}
}  // namespace uipc::info_stackless_detail

namespace uipc::backend::cuda
{
using namespace info_stackless_detail;

namespace
{
    __global__ void InfoStacklessBVH_initializeBuildState_kernel(
        int                             num_objs,
        cuda_tool::BufferView<uint32_t> flags,
        cuda_tool::BufferView<int>      ext_lca,
        cuda_tool::BufferView<uint32_t> depths,
        cuda_tool::BufferView<int32_t>  unsorted_ids)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx < num_objs - 1)
            flags(idx) = 0;
        if(idx < num_objs)
        {
            depths(idx)       = 0;
            unsorted_ids(idx) = idx;
        }
        if(idx <= num_objs)
            ext_lca(idx) = -1;
    }

    __global__ void InfoStacklessBVH_initializeQueryState_kernel(int num_objs,
                                                                 cuda_tool::BufferView<int> unsorted_ids)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx < num_objs)
            unsorted_ids(idx) = idx;
    }

    __global__ void InfoStacklessBVH_resetSceneBox_kernel(cuda_tool::Dense<AABB> out)
    {
        *out = AABB();
    }

    __global__ void InfoStacklessBVH_calcMaxBVFromBox_kernel(
        size_t size, cuda_tool::CBufferView<AABB> box, cuda_tool::Dense<AABB> out)
    {
        int                  idx = blockIdx.x * blockDim.x + threadIdx.x;
        __shared__ PlainAABB warp_boxes[K_BUILD_WARPS];
        int                  warp_tid = threadIdx.x & 31;
        int                  warp_id  = threadIdx.x >> 5;

        PlainAABB temp;
        if(idx < size)
        {
            temp = to_plain(box(idx));
        }
        else
        {
            constexpr float max_float = 3.402823466e+38F;
            temp._min = make_float3(max_float, max_float, max_float);
            temp._max = make_float3(-max_float, -max_float, -max_float);
        }

        float minx = temp._min.x, miny = temp._min.y, minz = temp._min.z;
        float maxx = temp._max.x, maxy = temp._max.y, maxz = temp._max.z;
        for(int offset = 16; offset > 0; offset >>= 1)
        {
            minx = mm_min(minx, __shfl_down_sync(0xffffffff, minx, offset));
            miny = mm_min(miny, __shfl_down_sync(0xffffffff, miny, offset));
            minz = mm_min(minz, __shfl_down_sync(0xffffffff, minz, offset));
            maxx = mm_max(maxx, __shfl_down_sync(0xffffffff, maxx, offset));
            maxy = mm_max(maxy, __shfl_down_sync(0xffffffff, maxy, offset));
            maxz = mm_max(maxz, __shfl_down_sync(0xffffffff, maxz, offset));
        }
        if(warp_tid == 0)
        {
            warp_boxes[warp_id]._min = make_float3(minx, miny, minz);
            warp_boxes[warp_id]._max = make_float3(maxx, maxy, maxz);
        }
        __syncthreads();

        if(warp_id == 0)
        {
            constexpr float max_float = 3.402823466e+38F;
            if(warp_tid < K_BUILD_WARPS)
                temp = warp_boxes[warp_tid];
            else
            {
                temp._min = make_float3(max_float, max_float, max_float);
                temp._max = make_float3(-max_float, -max_float, -max_float);
            }

            minx = temp._min.x;
            miny = temp._min.y;
            minz = temp._min.z;
            maxx = temp._max.x;
            maxy = temp._max.y;
            maxz = temp._max.z;
            for(int offset = 16; offset > 0; offset >>= 1)
            {
                minx = mm_min(minx, __shfl_down_sync(0xffffffff, minx, offset));
                miny = mm_min(miny, __shfl_down_sync(0xffffffff, miny, offset));
                minz = mm_min(minz, __shfl_down_sync(0xffffffff, minz, offset));
                maxx = mm_max(maxx, __shfl_down_sync(0xffffffff, maxx, offset));
                maxy = mm_max(maxy, __shfl_down_sync(0xffffffff, maxy, offset));
                maxz = mm_max(maxz, __shfl_down_sync(0xffffffff, maxz, offset));
            }
            if(warp_tid == 0)
            {
                atomic_minf(&out->min().x(), minx);
                atomic_minf(&out->min().y(), miny);
                atomic_minf(&out->min().z(), minz);
                atomic_maxf(&out->max().x(), maxx);
                atomic_maxf(&out->max().y(), maxy);
                atomic_maxf(&out->max().z(), maxz);
            }
        }
    }

    __global__ void InfoStacklessBVH_calcMCsFromBox_kernel(cuda_tool::CBufferView<AABB> box,
                                                           cuda_tool::CDense<AABB> scene,
                                                           cuda_tool::BufferView<uint32_t> codes,
                                                           int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto   bv        = box(idx);
        auto   center    = bv.center();
        float3 c         = make_float3(center.x(), center.y(), center.z());
        auto   scene_min = scene->min();
        float3 smin = make_float3(scene_min.x(), scene_min.y(), scene_min.z());
        auto   scene_size = scene->sizes();
        float3 off        = c - smin;
        float  nx  = scene_size.x() > 0.0f ? off.x / scene_size.x() : 0.0f;
        float  ny  = scene_size.y() > 0.0f ? off.y / scene_size.y() : 0.0f;
        float  nz  = scene_size.z() > 0.0f ? off.z / scene_size.z() : 0.0f;
        codes(idx) = morton3D(nx, ny, nz);
    }

    __global__ void InfoStacklessBVH_calcInverseMapping_kernel(
        cuda_tool::BufferView<int32_t> map, cuda_tool::BufferView<int32_t> inv, int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        inv(map(idx)) = idx;
    }

    __global__ void InfoStacklessBVH_buildPrimitivesFromBox_kernel(
        cuda_tool::BufferView<int>     _prim_idx,
        cuda_tool::BufferView<AABB>    _prim_box,
        cuda_tool::BufferView<int32_t> _prim_map,
        cuda_tool::BufferView<IndexT>  _ext_bid,
        cuda_tool::BufferView<IndexT>  _ext_cid,
        cuda_tool::CBufferView<IndexT> _bids,
        cuda_tool::CBufferView<IndexT> _cids,
        bool                           has_info,
        cuda_tool::CBufferView<AABB>   box,
        int                            n)
    {
        constexpr IndexT invalid = static_cast<IndexT>(-1);
        int              idx     = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        int new_idx        = _prim_map(idx);
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
    }

    __global__ void InfoStacklessBVH_calcExtNodeSplitMetrics_kernel(
        cuda_tool::BufferView<uint32_t> codes, cuda_tool::BufferView<int> metrics, int n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        // Equal Morton codes are ordered by rank (key = code << 32 | rank), otherwise
        // every tie gives metric 0 and the tree degenerates into a chain.
        metrics(idx) = idx != n - 1 ? (codes(idx) != codes(idx + 1) ?
                                           64 - __clz(codes(idx) ^ codes(idx + 1)) :
                                           32 - __clz(idx ^ (idx + 1))) :
                                      65;
    }

    __global__ void InfoStacklessBVH_buildIntNodes_kernel(
        int                             size,
        cuda_tool::BufferView<uint32_t> _depths,
        cuda_tool::BufferView<int>      _lvs_lca,
        cuda_tool::BufferView<int>      _lvs_metric,
        cuda_tool::BufferView<uint32_t> _lvs_par,
        cuda_tool::BufferView<AABB>     _lvs_box,
        cuda_tool::BufferView<IndexT>   _lvs_bid,
        cuda_tool::BufferView<IndexT>   _lvs_cid,
        cuda_tool::BufferView<int>      _tks_lc,
        cuda_tool::BufferView<int>      _tks_rc,
        cuda_tool::BufferView<int>      _tks_range_x,
        cuda_tool::BufferView<int>      _tks_range_y,
        cuda_tool::BufferView<uint32_t> _tks_mark,
        cuda_tool::BufferView<AABB>     _tks_box,
        cuda_tool::BufferView<IndexT>   _tks_bid,
        cuda_tool::BufferView<IndexT>   _tks_cid,
        cuda_tool::BufferView<uint32_t> _flag,
        cuda_tool::BufferView<int>      _tks_par)
    {
        constexpr IndexT invalid = static_cast<IndexT>(-1);
        int              idx     = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= size)
            return;

        int  l        = idx - 1;
        int  r        = idx;
        bool mark     = (l >= 0) ? (_lvs_metric(l) < _lvs_metric(r)) : false;
        int  cur      = mark ? l : r;
        _lvs_par(idx) = cur;
        if(_flag.total_size() == 0)
            return;

        if(mark)
        {
            _tks_rc(cur)      = idx;
            _tks_range_y(cur) = idx;
            atomicOr(&_tks_mark(cur), 0x00000002);
        }
        else
        {
            _tks_lc(cur)      = idx;
            _tks_range_x(cur) = idx;
            atomicOr(&_tks_mark(cur), 0x00000001);
        }
        __threadfence();

        while(atomicAdd(&_flag(cur), 1) == 1)
        {
            int      chl = _tks_lc(cur);
            int      chr = _tks_rc(cur);
            uint32_t m   = _tks_mark(cur);
            if(m & 1)
                _tks_box(cur) = _lvs_box(chl);
            else
                _tks_box(cur) = _tks_box(chl);
            if(m & 2)
                _tks_box(cur).extend(_lvs_box(chr));
            else
                _tks_box(cur).extend(_tks_box(chr));

            IndexT l_bid  = (m & 1) ? _lvs_bid(chl) : _tks_bid(chl);
            IndexT r_bid  = (m & 2) ? _lvs_bid(chr) : _tks_bid(chr);
            IndexT l_cid  = (m & 1) ? _lvs_cid(chl) : _tks_cid(chl);
            IndexT r_cid  = (m & 2) ? _lvs_cid(chr) : _tks_cid(chr);
            _tks_bid(cur) = (l_bid == r_bid) ? l_bid : invalid;
            _tks_cid(cur) = (l_cid == r_cid) ? l_cid : invalid;

            _tks_mark(cur) &= 0x00000007;
            l               = _tks_range_x(cur) - 1;
            r               = _tks_range_y(cur);
            _lvs_lca(l + 1) = cur;
            _depths(l + 1)++;
            mark = (l >= 0) ? (_lvs_metric(l) < _lvs_metric(r)) : false;
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

    __global__ void InfoStacklessBVH_calcIntNodeOrders_kernel(
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
                _tkMap(node) = id++;
        }
    }

    __global__ void InfoStacklessBVH_updateBvhExtNodeLinks_kernel(
        cuda_tool::BufferView<int>      _map,
        cuda_tool::BufferView<int>      _lcas,
        cuda_tool::BufferView<uint32_t> _pars,
        int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        _pars(idx) = _map(_pars(idx));
        int ori    = _lcas(idx);
        _lcas(idx) = (ori != -1) ? (_map(ori) << 1) : (idx << 1 | 1);
    }

    __global__ void InfoStacklessBVH_reorderNode_kernel(
        int                                           int_size,
        cuda_tool::BufferView<int>                    _lvs_lca,
        cuda_tool::BufferView<uint32_t>               _lvs_par,
        cuda_tool::BufferView<AABB>                   _lvs_box,
        cuda_tool::BufferView<IndexT>                 _lvs_bid,
        cuda_tool::BufferView<IndexT>                 _lvs_cid,
        cuda_tool::BufferView<int>                    _tk_map,
        cuda_tool::BufferView<int>                    _int_lc,
        cuda_tool::BufferView<int>                    _int_rc,
        cuda_tool::BufferView<int>                    _int_par,
        cuda_tool::BufferView<uint32_t>               _int_mark,
        cuda_tool::BufferView<int>                    _int_range_y,
        cuda_tool::BufferView<int>                    _self_max_rank,
        cuda_tool::BufferView<int>                    _refit_parent,
        cuda_tool::BufferView<int>                    _refit_right,
        cuda_tool::BufferView<AABB>                   _int_box,
        cuda_tool::BufferView<IndexT>                 _int_bid,
        cuda_tool::BufferView<IndexT>                 _int_cid,
        cuda_tool::BufferView<InfoStacklessBVH::Node> _nodes,
        int                                           count)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= count)
            return;
        InfoStacklessBVH::Node leaf;
        leaf.lc    = -1;
        int escape = _lvs_lca(idx + 1);
        if(escape == -1)
            leaf.escape = -1;
        else
        {
            int b_leaf = escape & 1;
            escape >>= 1;
            leaf.escape = escape + (b_leaf ? int_size : 0);
        }
        leaf.bound             = _lvs_box(idx);
        leaf.bid               = _lvs_bid(idx);
        leaf.cid               = _lvs_cid(idx);
        _nodes(idx + int_size) = leaf;
        _refit_parent(idx + int_size) =
            int_size == 0 ? -1 : static_cast<int>(_lvs_par(idx));

        if(idx >= int_size)
            return;

        InfoStacklessBVH::Node n;
        int                    new_id = _tk_map(idx);
        uint32_t               m      = _int_mark(idx);
        _self_max_rank(new_id)        = _int_range_y(idx);
        n.lc = (m & 1) ? _int_lc(idx) + int_size : _tk_map(_int_lc(idx));
        _refit_right(new_id) = (m & 2) ? _int_rc(idx) + int_size : _tk_map(_int_rc(idx));
        int old_parent        = _int_par(idx);
        _refit_parent(new_id) = old_parent == -1 ? -1 : _tk_map(old_parent);
        n.bound               = _int_box(idx);
        int ie                = _lvs_lca(_int_range_y(idx) + 1);
        if(ie == -1)
            n.escape = -1;
        else
        {
            int b_leaf = ie & 1;
            ie >>= 1;
            n.escape = ie + (b_leaf ? int_size : 0);
        }
        n.bid          = _int_bid(idx);
        n.cid          = _int_cid(idx);
        _nodes(new_id) = n;
    }

    __global__ void InfoStacklessBVH_refit_kernel(
        int                                           int_size,
        cuda_tool::CBufferView<AABB>                  _aabbs,
        cuda_tool::CBufferView<IndexT>                _bids,
        cuda_tool::CBufferView<IndexT>                _cids,
        cuda_tool::CBufferView<int>                   _lvs_idx,
        cuda_tool::BufferView<AABB>                   _lvs_box,
        cuda_tool::BufferView<IndexT>                 _lvs_bid,
        cuda_tool::BufferView<IndexT>                 _lvs_cid,
        cuda_tool::BufferView<InfoStacklessBVH::Node> _nodes,
        cuda_tool::CBufferView<int>                   _parent,
        cuda_tool::CBufferView<int>                   _right,
        cuda_tool::BufferView<int>                    _arrivals,
        int                                           n)
    {
        constexpr IndexT invalid = static_cast<IndexT>(-1);
        int              rank    = blockIdx.x * blockDim.x + threadIdx.x;
        if(rank >= n)
            return;

        int raw_id  = _lvs_idx(rank);
        int leaf_id = int_size + rank;

        InfoStacklessBVH::Node leaf = _nodes(leaf_id);
        leaf.bound                  = _aabbs(raw_id);
        leaf.bid                    = _bids(raw_id);
        leaf.cid                    = _cids(raw_id);
        _nodes(leaf_id)             = leaf;
        _lvs_box(rank)              = leaf.bound;
        _lvs_bid(rank)              = leaf.bid;
        _lvs_cid(rank)              = leaf.cid;

        // Publish the leaf before announcing its arrival. The first child at
        // each parent stops; the second observes both children, publishes the
        // merged node, and carries completion toward the root.
        __threadfence();
        int parent = _parent(leaf_id);
        while(parent != -1)
        {
            if(atomicAdd(&_arrivals(parent), 1) == 0)
                break;

            __threadfence();
            auto node  = _nodes(parent);
            auto left  = _nodes(node.lc);
            auto right = _nodes(_right(parent));
            node.bound = left.bound;
            node.bound.extend(right.bound);
            node.bid       = left.bid == right.bid ? left.bid : invalid;
            node.cid       = left.cid == right.cid ? left.cid : invalid;
            _nodes(parent) = node;

            __threadfence();
            parent = _parent(parent);
        }
    }

    template <typename NodeCull, typename PairPred>
    __global__ void InfoStacklessBVH_stacklessSelf_kernel(
        int                                           Size,
        cuda_tool::CBufferView<AABB>                  _box,
        int                                           intSize,
        int                                           numObjs,
        cuda_tool::BufferView<int>                    _lvs_idx,
        cuda_tool::BufferView<InfoStacklessBVH::Node> _nodes,
        cuda_tool::BufferView<int>                    _self_max_rank,
        cuda_tool::CBufferView<IndexT>                _bids,
        cuda_tool::CBufferView<IndexT>                _cids,
        bool                                          has_info,
        cuda_tool::Dense<int>                         resCounter,
        cuda_tool::BufferView<Vector2i>               res,
        NodeCull                                      node_cull,
        PairPred                                      pair_pred)
    {
        constexpr IndexT invalid = static_cast<IndexT>(-1);
        int              tid     = blockIdx.x * blockDim.x + threadIdx.x;
        bool             active  = tid < Size;
        int              idx     = -1;
        AABB             bv;
        if(active)
        {
            idx = _lvs_idx(tid);
            bv  = _box(idx);
        }

        // -----------------------------------------------------------------
        // SMem: pre-load query bid/cid once per thread, before hot loop.
        // Shared storage follows the independently swept Self CTA and queue.
        // -----------------------------------------------------------------
        __shared__ IndexT s_qbid[K_SELF_THREADS];
        __shared__ IndexT s_qcid[K_SELF_THREADS];

        s_qbid[threadIdx.x] = (active && has_info) ? _bids(idx) : invalid;
        s_qcid[threadIdx.x] = (active && has_info) ? _cids(idx) : invalid;

        __shared__ int2 shared_res[K_SELF_MAX_RES_PER_BLOCK];
        __shared__ int  shared_counter;
        __shared__ int  shared_global_idx;
        if(threadIdx.x == 0)
            shared_counter = 0;

        int       st       = 0;
        const int max_iter = numObjs * 2;
        while(true)
        {
            // First __syncthreads also ensures SMem pre-loads above are
            // visible to all threads in the block (though each thread
            // only reads its own slot: s_qbid[threadIdx.x]).
            __syncthreads();
            if(active)
            {
                int inner_i = 0;
                for(; inner_i < max_iter; ++inner_i)
                {
                    if(st == -1)
                        break;
                    // Self traversal only accepts leaves with Morton rank > tid.
                    // An internal subtree whose maximum rank cannot pass that
                    // gate is the duplicate half and can be skipped wholesale.
                    if(st < intSize && _self_max_rank(st) <= tid)
                    {
                        st = _nodes(st).escape;
                        continue;
                    }
                    auto node = _nodes(st);
                    if(!node.bound.intersects(bv))
                    {
                        st = node.escape;
                        continue;
                    }
                    if(!node_cull(InfoStacklessBVH::NodePredInfo{
                           idx, s_qbid[threadIdx.x], s_qcid[threadIdx.x], node.bid, node.cid}))
                    {
                        st = node.escape;
                        continue;
                    }
                    if(node.lc == -1)
                    {
                        if(tid < st - intSize)
                        {
                            int  leaf_raw = _lvs_idx(st - intSize);
                            bool q_first  = (idx < leaf_raw);
                            auto pair     = ordered_pair(idx, leaf_raw);
                            InfoStacklessBVH::LeafPredInfo leaf_info{
                                pair.x,
                                pair.y,
                                q_first ? s_qbid[threadIdx.x] : node.bid,
                                q_first ? s_qcid[threadIdx.x] : node.cid,
                                q_first ? node.bid : s_qbid[threadIdx.x],
                                q_first ? node.cid : s_qcid[threadIdx.x]};
                            if(pair_pred(leaf_info))
                            {
                                int sidx = atomicAdd(&shared_counter, 1);
                                if(sidx >= K_SELF_MAX_RES_PER_BLOCK)
                                    break;
                                shared_res[sidx] = pair;
                            }
                        }
                        st = node.escape;
                    }
                    else
                        st = node.lc;
                }
                UIPC_KERNEL_ASSERT(inner_i < max_iter, "Exceeded max stackless iteration");
            }
            __syncthreads();
            int total = min(shared_counter, K_SELF_MAX_RES_PER_BLOCK);
            if(threadIdx.x == 0)
                shared_global_idx = atomicAdd(resCounter.data(), total);
            __syncthreads();
            int gidx = shared_global_idx;
            if(threadIdx.x == 0)
                shared_counter = 0;
            bool done = total < K_SELF_MAX_RES_PER_BLOCK;
            safe_copy_to(shared_res, total, res.data(), gidx, static_cast<int>(res.total_size()));
            if(done)
                break;
        }
    }

    template <typename NodeCull, typename PairPred>
    __global__ void InfoStacklessBVH_stacklessOther_kernel(
        int                                           Size,
        cuda_tool::CBufferView<AABB>                  _box,
        cuda_tool::CBufferView<int>                   sortedIdx,
        int                                           intSize,
        int                                           numObjs,
        cuda_tool::BufferView<int>                    _lvs_idx,
        cuda_tool::BufferView<InfoStacklessBVH::Node> _nodes,
        cuda_tool::CBufferView<IndexT>                _qbids,
        cuda_tool::CBufferView<IndexT>                _qcids,
        bool                                          qhas_info,
        cuda_tool::Dense<int>                         resCounter,
        cuda_tool::BufferView<Vector2i>               res,
        NodeCull                                      node_cull,
        PairPred                                      pair_pred)
    {
        constexpr IndexT invalid = static_cast<IndexT>(-1);
        int              tid     = blockIdx.x * blockDim.x + threadIdx.x;
        bool             active  = tid < Size;
        int              idx     = -1;
        AABB             bv;
        if(active)
        {
            idx = sortedIdx(tid);
            bv  = _box(idx);
        }

        // -----------------------------------------------------------------
        // SMem: pre-load per-query bid/cid before the traversal loop.
        // -----------------------------------------------------------------
        __shared__ IndexT s_qbid[K_OTHER_THREADS];
        __shared__ IndexT s_qcid[K_OTHER_THREADS];

        s_qbid[threadIdx.x] = (active && qhas_info) ? _qbids(idx) : invalid;
        s_qcid[threadIdx.x] = (active && qhas_info) ? _qcids(idx) : invalid;

        __shared__ int2 shared_res[K_OTHER_MAX_RES_PER_BLOCK];
        __shared__ int  shared_counter;
        __shared__ int  shared_global_idx;
        if(threadIdx.x == 0)
            shared_counter = 0;

        int       st       = 0;
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
                    if(!node_cull(InfoStacklessBVH::NodePredInfo{
                           idx, s_qbid[threadIdx.x], s_qcid[threadIdx.x], node.bid, node.cid}))
                    {
                        st = node.escape;
                        continue;
                    }
                    if(node.lc == -1)
                    {
                        auto pair = int2{idx, _lvs_idx(st - intSize)};
                        // query side: SMem pre-loaded; leaf side: node.bid/cid
                        InfoStacklessBVH::LeafPredInfo leaf_info{
                            pair.x,
                            pair.y,
                            s_qbid[threadIdx.x],
                            s_qcid[threadIdx.x],
                            node.bid,
                            node.cid};
                        if(pair_pred(leaf_info))
                        {
                            int sidx = atomicAdd(&shared_counter, 1);
                            if(sidx >= K_OTHER_MAX_RES_PER_BLOCK)
                                break;
                            shared_res[sidx] = pair;
                        }
                        st = node.escape;
                    }
                    else
                        st = node.lc;
                }
                UIPC_KERNEL_ASSERT(inner_i < max_iter, "Exceeded max stackless iteration");
            }

            __syncthreads();
            int total = min(shared_counter, K_OTHER_MAX_RES_PER_BLOCK);
            if(threadIdx.x == 0)
                shared_global_idx = atomicAdd(resCounter.data(), total);
            __syncthreads();
            int gidx = shared_global_idx;
            if(threadIdx.x == 0)
                shared_counter = 0;
            __syncthreads();
            bool done = total < K_OTHER_MAX_RES_PER_BLOCK;
            safe_copy_to(shared_res, total, res.data(), gidx, static_cast<int>(res.total_size()));
            if(done)
                break;
        }
    }

    // One query of a warp-cooperative Other traversal: its box and ids.
    struct OtherQuerySlot
    {
        float3 lo;
        float3 hi;
        int    idx;
        IndexT bid;
        IndexT cid;
    };

    // Warp-cooperative Other traversal. A warp owns `queries_per_warp` queries
    // and shares their traversal among its 32 lanes. A lane's task is the part
    // of one query's stackless traversal that starts at node `st` and ends
    // when the escape chain reaches `end`. When a lane descends into the left
    // child of an internal node, it may donate the remainder (right child up
    // to `end`) to a per-warp task pool and keep only the left subtree, whose
    // escape chain ends exactly at that right child; idle lanes take pooled
    // tasks. The tasks partition each query's traversal, so every (query, node)
    // pair is visited exactly once, as in the per-lane kernel. Leaves that pass
    // the box and node tests go to a per-warp queue; once it holds 32 entries
    // all lanes evaluate the leaf predicate together. Accepted pairs go
    // straight to global memory through one warp-aggregated atomic, and the
    // counter keeps counting past the output size for the caller's retry.
    template <typename PairPred>
    __device__ void InfoStacklessBVH_drainLeafQueue(int         count,
                                                    int         first,
                                                    const int4* leaves,
                                                    const OtherQuerySlot* slots,
                                                    cuda_tool::BufferView<int> lvs_idx,
                                                    cuda_tool::Dense<int> resCounter,
                                                    cuda_tool::BufferView<Vector2i> res,
                                                    PairPred& pair_pred)
    {
        constexpr unsigned full_mask   = 0xFFFFFFFFu;
        const int          lane        = threadIdx.x & 31;
        const unsigned     lanes_below = (1u << lane) - 1u;

        int  pair_i = -1;
        int  pair_j = -1;
        bool accept = false;
        int4 entry  = make_int4(-1, -1, -1, -1);
        if(lane < count)
            entry = leaves[first + lane];
        // The next push may overwrite the drained slots.
        __syncwarp();
        if(lane < count)
        {
            const OtherQuerySlot& q = slots[entry.x];
            pair_i                  = q.idx;
            pair_j                  = lvs_idx(entry.y);
            accept                  = pair_pred(InfoStacklessBVH::LeafPredInfo{
                pair_i, pair_j, q.bid, q.cid, entry.z, entry.w});
        }
        const unsigned accepted = __ballot_sync(full_mask, accept);
        if(accepted)
        {
            int base = 0;
            if(lane == 0)
                base = atomicAdd(resCounter.data(), __popc(accepted));
            base        = __shfl_sync(full_mask, base, 0);
            const int g = base + __popc(accepted & lanes_below);
            if(accept && g < static_cast<int>(res.total_size()))
                res(g) = Vector2i{pair_i, pair_j};
        }
    }

    // No minimum of blocks per SM, so no register cap: the compiler may keep
    // a pair predicate as a function of its own, and ptxas rejects a capped
    // kernel's call to a function that needs more registers than the cap.
    template <typename NodeCull, typename PairPred>
    __global__ void __launch_bounds__(K_OTHER_THREADS)
        InfoStacklessBVH_stacklessOtherWarp_kernel(
            int                                           Size,
            cuda_tool::CBufferView<AABB>                  _box,
            cuda_tool::CBufferView<int>                   sortedIdx,
            int                                           intSize,
            int                                           numObjs,
            cuda_tool::BufferView<int>                    _lvs_idx,
            cuda_tool::BufferView<InfoStacklessBVH::Node> _nodes,
            cuda_tool::BufferView<int>                    _right_child,
            cuda_tool::CBufferView<IndexT>                _qbids,
            cuda_tool::CBufferView<IndexT>                _qcids,
            bool                                          qhas_info,
            int                                           queries_per_warp,
            cuda_tool::Dense<int>                         resCounter,
            cuda_tool::BufferView<Vector2i>               res,
            NodeCull                                      node_cull,
            PairPred                                      pair_pred)
    {
        constexpr IndexT   invalid     = static_cast<IndexT>(-1);
        constexpr unsigned full_mask   = 0xFFFFFFFFu;
        constexpr int      warps       = K_OTHER_THREADS / 32;
        const int          lane        = threadIdx.x & 31;
        const int          warp        = threadIdx.x >> 5;
        const unsigned     lanes_below = (1u << lane) - 1u;

        __shared__ OtherQuerySlot s_slots[warps][32];
        __shared__ int4           s_tasks[warps][K_OTHER_WARP_TASKS];
        __shared__ int4           s_leaves[warps][K_OTHER_WARP_QUEUE];
        OtherQuerySlot*           slots  = s_slots[warp];
        int4*                     tasks  = s_tasks[warp];
        int4*                     leaves = s_leaves[warp];

        // This lane's task: query slot, next node and end node (-1: done).
        int    slot = -1;
        int    st   = -1;
        int    end  = -1;
        int    idx  = -1;
        IndexT qbid = invalid;
        IndexT qcid = invalid;
        AABB   bv;

        const int first_query = (blockIdx.x * warps + warp) * queries_per_warp;
        if(lane < queries_per_warp && first_query + lane < Size)
        {
            idx  = sortedIdx(first_query + lane);
            bv   = _box(idx);
            qbid = qhas_info ? _qbids(idx) : invalid;
            qcid = qhas_info ? _qcids(idx) : invalid;
            slots[lane] =
                OtherQuerySlot{make_float3(bv.min().x(), bv.min().y(), bv.min().z()),
                               make_float3(bv.max().x(), bv.max().y(), bv.max().z()),
                               idx,
                               qbid,
                               qcid};
            slot = lane;
            st   = 0;
        }
        __syncwarp();

        // Warp-uniform pool size, leaf queue length and node visit budget
        // (a query visits each node at most once).
        int pooled  = 0;
        int pending = 0;
        long long budget = static_cast<long long>(queries_per_warp) * 2 * numObjs;
        while(true)
        {
            // Idle lanes take the most recently pooled tasks.
            const unsigned idle  = __ballot_sync(full_mask, st == -1);
            const int      taken = min(__popc(idle), pooled);
            if(taken > 0)
            {
                const int rank = __popc(idle & lanes_below);
                if(st == -1 && rank < taken)
                {
                    const int4 t           = tasks[pooled - 1 - rank];
                    slot                   = t.x;
                    st                     = t.y;
                    end                    = t.z;
                    const OtherQuerySlot q = slots[slot];
                    idx                    = q.idx;
                    qbid                   = q.bid;
                    qcid                   = q.cid;
                    bv = AABB{Eigen::Vector3f{q.lo.x, q.lo.y, q.lo.z},
                              Eigen::Vector3f{q.hi.x, q.hi.y, q.hi.z}};
                }
                pooled -= taken;
                // Donations below may overwrite the taken entries.
                __syncwarp();
            }

            const unsigned busy = __ballot_sync(full_mask, st != -1);
            budget -= __popc(busy);
            if(busy == 0 || budget < 0)
            {
                UIPC_KERNEL_ASSERT(busy == 0, "Exceeded max stackless iteration");
                break;
            }

            // Every busy lane visits one node of its task.
            int4 hit   = make_int4(-1, -1, -1, -1);
            int  right = -1;
            if(st != -1)
            {
                const auto node = _nodes(st);
                if(!node.bound.intersects(bv)
                   || !node_cull(InfoStacklessBVH::NodePredInfo{
                       idx, qbid, qcid, node.bid, node.cid}))
                {
                    st = node.escape == end ? -1 : node.escape;
                }
                else if(node.lc == -1)
                {
                    hit = make_int4(slot, st - intSize, node.bid, node.cid);
                    st  = node.escape == end ? -1 : node.escape;
                }
                else
                {
                    right = _right_child(st);
                    st    = node.lc;
                }
            }

            // Donate the right child up to `end` while the pool has room; the
            // donor then ends its task at that right child.
            const unsigned donors = __ballot_sync(full_mask, right != -1);
            if(donors)
            {
                const int room = K_OTHER_WARP_TASKS - pooled;
                const int rank = __popc(donors & lanes_below);
                if(right != -1 && rank < room)
                {
                    tasks[pooled + rank] = make_int4(slot, right, end, 0);
                    end                  = right;
                }
                pooled += min(__popc(donors), room);
            }

            const unsigned hits = __ballot_sync(full_mask, hit.x >= 0);
            if(hit.x >= 0)
                leaves[pending + __popc(hits & lanes_below)] = hit;
            pending += __popc(hits);
            if(pending >= 32)
            {
                __syncwarp();
                pending -= 32;
                InfoStacklessBVH_drainLeafQueue<PairPred>(
                    32, pending, leaves, slots, _lvs_idx, resCounter, res, pair_pred);
            }
            __syncwarp();
        }

        while(pending > 0)
        {
            __syncwarp();
            const int count = min(pending, 32);
            pending -= count;
            InfoStacklessBVH_drainLeafQueue<PairPred>(
                count, pending, leaves, slots, _lvs_idx, resCounter, res, pair_pred);
        }
    }
}  // namespace

// ---------------------------------------------------------------------------
// Build and refit pipeline
// ---------------------------------------------------------------------------

inline void InfoStacklessBVH::Impl::calcMaxBVFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                                     cuda_tool::VarView<AABB> scene_box,
                                                     cudaStream_t stream)
{
    InfoStacklessBVH_resetSceneBox_kernel<<<1, 1, 0, stream>>>(scene_box.viewer());

    if(aabbs.size() == 0)
        return;

    auto num  = aabbs.size();
    auto grid = (num + K_BUILD_THREADS - 1) / K_BUILD_THREADS;

    if(grid > 0)
        InfoStacklessBVH_calcMaxBVFromBox_kernel<<<grid, K_BUILD_THREADS, 0, stream>>>(
            aabbs.size(), aabbs, scene_box.viewer());
}

inline void InfoStacklessBVH::Impl::calcMCsFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                                   cuda_tool::CVarView<AABB> scene_box,
                                                   cuda_tool::BufferView<uint32_t> codes,
                                                   cudaStream_t stream)
{
    auto k = InfoStacklessBVH_calcMCsFromBox_kernel;
    int  n = static_cast<int>(aabbs.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            aabbs, scene_box.viewer(), codes, n);
}

inline void InfoStacklessBVH::Impl::calcInverseMapping(cudaStream_t stream)
{
    auto k = InfoStacklessBVH_calcInverseMapping_kernel;
    int  n = static_cast<int>(sorted_id.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            sorted_id.view(), primMap.view(), n);
}

inline void InfoStacklessBVH::Impl::buildPrimitivesFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                                           cudaStream_t stream)
{
    bool has_info = bids.size() == aabbs.size() && cids.size() == aabbs.size();
    auto k        = InfoStacklessBVH_buildPrimitivesFromBox_kernel;
    int  n        = static_cast<int>(aabbs.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            ext_idx.view(),
            ext_aabb.view(),
            primMap.view(),
            ext_bid.view(),
            ext_cid.view(),
            bids,
            cids,
            has_info,
            aabbs,
            n);
}

inline void InfoStacklessBVH::Impl::calcExtNodeSplitMetrics(cudaStream_t stream)
{
    auto k = InfoStacklessBVH_calcExtNodeSplitMetrics_kernel;
    int  n = static_cast<int>(sorted_mtcode.size());
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            sorted_mtcode.view(), metric.view(), n);
}

inline void InfoStacklessBVH::Impl::buildIntNodes(int size, cudaStream_t stream)
{
    auto grid = (size + 255) / 256;
    if(grid > 0)
        InfoStacklessBVH_buildIntNodes_kernel<<<grid, 256, 0, stream>>>(
            size,
            count.view(),
            ext_lca.view(),
            metric.view(),
            ext_par.view(),
            ext_aabb.view(),
            ext_bid.view(),
            ext_cid.view(),
            int_lc.view(),
            int_rc.view(),
            int_range_x.view(),
            int_range_y.view(),
            int_mark.view(),
            int_aabb.view(),
            int_bid.view(),
            int_cid.view(),
            flags.view(),
            int_par.view());
}

inline void InfoStacklessBVH::Impl::calcIntNodeOrders(int size, cudaStream_t stream)
{
    auto k = InfoStacklessBVH_calcIntNodeOrders_kernel;
    if(size > 0)
        k<<<cuda_tool::best_grid_dim(size, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            int_lc.view(),
            ext_lca.view(),
            count.view(),
            offsetTable.view(),
            tkMap.view(),
            size);
}

inline void InfoStacklessBVH::Impl::updateBvhExtNodeLinks(int size, cudaStream_t stream)
{
    if(flags.size() == 0)
        return;
    auto k = InfoStacklessBVH_updateBvhExtNodeLinks_kernel;
    if(size > 0)
        k<<<cuda_tool::best_grid_dim(size, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            tkMap.view(), ext_lca.view(), ext_par.view(), size);
}

inline void InfoStacklessBVH::Impl::reorderNode(int int_size, cudaStream_t stream)
{
    auto k = InfoStacklessBVH_reorderNode_kernel;
    int  n = int_size + 1;
    if(n > 0)
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            int_size,
            ext_lca.view(),
            ext_par.view(),
            ext_aabb.view(),
            ext_bid.view(),
            ext_cid.view(),
            tkMap.view(),
            int_lc.view(),
            int_rc.view(),
            int_par.view(),
            int_mark.view(),
            int_range_y.view(),
            self_max_rank.view(),
            refit_parent.view(),
            refit_right_child.view(),
            int_aabb.view(),
            int_bid.view(),
            int_cid.view(),
            nodes.view(),
            n);
}

inline void InfoStacklessBVH::Impl::propagateInformativeMetadata(int) {}

inline void InfoStacklessBVH::Impl::build(cuda_tool::CBufferView<AABB>   aabbs,
                                          cuda_tool::CBufferView<IndexT> _bids,
                                          cuda_tool::CBufferView<IndexT> _cids,
                                          cudaStream_t                   stream)
{
    objs              = aabbs;
    bids              = _bids;
    cids              = _cids;
    auto num_objs     = aabbs.size();
    auto num_internal = num_objs > 0 ? num_objs - 1 : 0;
    self_max_rank.resize_discard(num_internal, stream);
    if(num_objs == 0)
        return;

    auto num_nodes = num_objs * 2 - 1;
    mtcode.resize(num_objs, stream);
    sorted_mtcode.resize(num_objs, stream);
    sorted_id.resize(num_objs, stream);
    primMap.resize(num_objs, stream);
    ext_aabb.resize(num_objs, stream);
    ext_idx.resize(num_objs, stream);
    ext_lca.resize(num_objs + 1, stream);
    ext_par.resize(num_objs, stream);
    ext_bid.resize(num_objs, stream);
    ext_cid.resize(num_objs, stream);
    metric.resize(num_objs, stream);
    tkMap.resize(num_objs, stream);
    offsetTable.resize(num_objs, stream);
    count.resize(num_objs, stream);
    flags.resize(num_internal, stream);
    int_lc.resize(num_internal, stream);
    int_rc.resize(num_internal, stream);
    int_par.resize(num_internal, stream);
    int_range_x.resize(num_internal, stream);
    int_range_y.resize(num_internal, stream);
    int_mark.resize(num_internal, stream);
    int_aabb.resize(num_internal, stream);
    int_bid.resize(num_internal, stream);
    int_cid.resize(num_internal, stream);
    nodes.resize(num_nodes, stream);
    refit_parent.resize_discard(num_nodes, stream);
    refit_right_child.resize_discard(num_internal, stream);
    refit_arrivals.resize_discard(num_internal, stream);

    auto init = InfoStacklessBVH_initializeBuildState_kernel;
    auto n    = static_cast<int>(num_objs);
    init<<<cuda_tool::best_grid_dim(n + 1, init), cuda_tool::best_block_dim(init), 0, stream>>>(
        n, flags.view(), ext_lca.view(), count.view(), primMap.view());

    calcMaxBVFromBox(aabbs, scene_box.view(), stream);
    calcMCsFromBox(aabbs, scene_box.view(), mtcode.view(), stream);
    // The cuda_tool CUB wrappers share persistent scratch per CUDA stream;
    // repeated BVH builds query the required size but do not reallocate it.
    cuda_tool::DeviceRadixSort(stream).SortPairs(
        mtcode.data(), sorted_mtcode.data(), primMap.data(), sorted_id.data(), n);
    calcInverseMapping(stream);
    buildPrimitivesFromBox(aabbs, stream);
    calcExtNodeSplitMetrics(stream);
    buildIntNodes(num_objs, stream);
    cuda_tool::DeviceScan(stream).ExclusiveSum(count.data(), offsetTable.data(), n);
    calcIntNodeOrders(num_objs, stream);
    updateBvhExtNodeLinks(num_objs, stream);
    reorderNode(num_internal, stream);
}

inline bool InfoStacklessBVH::Impl::refit(cuda_tool::CBufferView<AABB>   aabbs,
                                          cuda_tool::CBufferView<IndexT> _bids,
                                          cuda_tool::CBufferView<IndexT> _cids,
                                          cudaStream_t                   stream)
{
    auto num_objs = aabbs.size();
    if(num_objs != objs.size() || _bids.size() != num_objs || _cids.size() != num_objs)
        return false;

    if(num_objs == 0)
    {
        objs = aabbs;
        bids = _bids;
        cids = _cids;
        return true;
    }

    auto num_internal = num_objs - 1;
    auto num_nodes    = num_objs * 2 - 1;
    if(nodes.size() != num_nodes || ext_idx.size() != num_objs
       || ext_aabb.size() != num_objs || ext_bid.size() != num_objs
       || ext_cid.size() != num_objs || refit_parent.size() != num_nodes
       || refit_right_child.size() != num_internal || refit_arrivals.size() != num_internal)
        return false;

    objs = aabbs;
    bids = _bids;
    cids = _cids;

    cuda_tool::BufferLaunch(stream).fill(refit_arrivals.view(), 0);

    // 128-thread blocks: the occupancy-maximizing block (768 threads) holds
    // its SMs through the whole bottom-up chain and keeps the concurrently
    // refitted trees off them. Any block size merges the same boxes.
    constexpr int RefitBlock = 128;
    auto          k          = InfoStacklessBVH_refit_kernel;
    auto          n          = static_cast<int>(num_objs);
    k<<<(n + RefitBlock - 1) / RefitBlock, RefitBlock, 0, stream>>>(
        static_cast<int>(num_internal),
        aabbs,
        _bids,
        _cids,
        ext_idx.view(),
        ext_aabb.view(),
        ext_bid.view(),
        ext_cid.view(),
        nodes.view(),
        refit_parent.view(),
        refit_right_child.view(),
        refit_arrivals.view(),
        n);
    return true;
}

// ---------------------------------------------------------------------------
// OPTIMIZED: stacklessSelf
//   Pre-loads query_bid and query_cid for each thread into shared memory
//   before the traversal loop. The node_cull functor receives a NodePredInfo
//   with query_bid/query_cid already filled from SMem — no global reads
//   inside the hot loop.
// ---------------------------------------------------------------------------
template <typename NodeCull, typename PairPred>
void InfoStacklessBVH::Impl::stacklessSelf(NodeCull                node_cull,
                                           PairPred                pair_pred,
                                           cuda_tool::VarView<int> cpNum,
                                           cuda_tool::BufferView<Vector2i> buffer,
                                           cudaStream_t stream)
{
    auto num_query = static_cast<int>(ext_aabb.size());
    auto num_objs  = num_query;
    auto grid      = (num_query + K_SELF_THREADS - 1) / K_SELF_THREADS;

    bool has_info = bids.size() == (size_t)num_objs && cids.size() == (size_t)num_objs;

    if(grid > 0)
        InfoStacklessBVH_stacklessSelf_kernel<NodeCull, PairPred>
            <<<grid, K_SELF_THREADS, 0, stream>>>(num_query,
                                                  objs,
                                                  num_objs - 1,
                                                  num_objs,
                                                  ext_idx.view(),
                                                  nodes.view(),
                                                  self_max_rank.view(),
                                                  bids,
                                                  cids,
                                                  has_info,
                                                  cpNum.viewer(),
                                                  buffer,
                                                  node_cull,
                                                  pair_pred);
}

// ---------------------------------------------------------------------------
// OPTIMIZED: stacklessOther
//   Takes explicit query_bids / query_cids buffers and pre-loads them into
//   shared memory. node_cull receives a NodePredInfo with query_bid/query_cid
//   already filled from SMem — no global reads inside the hot loop.
//   config.warp_cooperative_query (default) launches the warp-cooperative
//   kernel instead, which shares each warp's queries among its lanes and
//   batches the leaf predicates.
// ---------------------------------------------------------------------------
template <typename NodeCull, typename PairPred>
void InfoStacklessBVH::Impl::stacklessOther(NodeCull node_cull,
                                            PairPred pair_pred,
                                            cuda_tool::CBufferView<AABB> query_aabbs,
                                            cuda_tool::CBufferView<IndexT> query_bids,
                                            cuda_tool::CBufferView<IndexT> query_cids,
                                            cuda_tool::CBufferView<int> query_sorted_id,
                                            cuda_tool::VarView<int> cpNum,
                                            cuda_tool::BufferView<Vector2i> buffer,
                                            cudaStream_t stream)
{
    auto num_query = static_cast<int>(query_aabbs.size());
    auto num_objs  = static_cast<int>(ext_aabb.size());
    auto grid      = (num_query + K_OTHER_THREADS - 1) / K_OTHER_THREADS;

    bool qhas_info = query_bids.size() == (size_t)num_query
                     && query_cids.size() == (size_t)num_query;

    if(grid > 0 && config.warp_cooperative_query)
    {
        // Queries per warp: the largest power of two up to 32 that still
        // leaves 32 warps per SM, so few queries get many lanes each.
        int queries_per_warp = config.queries_per_warp;
        if(queries_per_warp <= 0)
        {
            int device = 0, sm_count = 1;
            CUDA_TOOL_CHECK(cudaGetDevice(&device));
            CUDA_TOOL_CHECK(cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount, device));
            const long long target = 32LL * std::max(sm_count, 1);
            queries_per_warp       = 1;
            while(queries_per_warp < 32
                  && (num_query + 2LL * queries_per_warp - 1) / (2LL * queries_per_warp) >= target)
                queries_per_warp *= 2;
        }
        queries_per_warp    = std::min(queries_per_warp, 32);
        constexpr int warps = K_OTHER_THREADS / 32;
        const int n_warps = (num_query + queries_per_warp - 1) / queries_per_warp;
        const int n_blocks = (n_warps + warps - 1) / warps;
        InfoStacklessBVH_stacklessOtherWarp_kernel<NodeCull, PairPred>
            <<<n_blocks, K_OTHER_THREADS, 0, stream>>>(num_query,
                                                       query_aabbs,
                                                       query_sorted_id,
                                                       num_objs - 1,
                                                       num_objs,
                                                       ext_idx.view(),
                                                       nodes.view(),
                                                       refit_right_child.view(),
                                                       query_bids,
                                                       query_cids,
                                                       qhas_info,
                                                       queries_per_warp,
                                                       cpNum.viewer(),
                                                       buffer,
                                                       node_cull,
                                                       pair_pred);
    }
    else if(grid > 0)
        InfoStacklessBVH_stacklessOther_kernel<NodeCull, PairPred>
            <<<grid, K_OTHER_THREADS, 0, stream>>>(num_query,
                                                   query_aabbs,
                                                   query_sorted_id,
                                                   num_objs - 1,
                                                   num_objs,
                                                   ext_idx.view(),
                                                   nodes.view(),
                                                   query_bids,
                                                   query_cids,
                                                   qhas_info,
                                                   cpNum.viewer(),
                                                   buffer,
                                                   node_cull,
                                                   pair_pred);
}

// ---------------------------------------------------------------------------
// Public API — same signatures as InfoStacklessBVH
// ---------------------------------------------------------------------------

inline InfoStacklessBVH::InfoStacklessBVH(cuda_tool::Stream& stream) noexcept
{
    (void)stream;
}

inline void InfoStacklessBVH::QueryBuffer::build(cuda_tool::CBufferView<AABB> aabbs,
                                                 bool         reuse_order,
                                                 cudaStream_t stream)
{
    // Morton order schedules traversal only; pair predicates still consume
    // current AABBs and primitive IDs. During one Line Search the query
    // identity and count are stable, so a cached permutation remains valid.
    if(reuse_order && m_querySortedId.size() == aabbs.size())
        return;

    m_queryMtCode.resize(aabbs.size(), stream);
    m_querySortedMtCode.resize(aabbs.size(), stream);
    m_queryId.resize(aabbs.size(), stream);
    m_querySortedId.resize(aabbs.size(), stream);
    auto init = InfoStacklessBVH_initializeQueryState_kernel;
    auto n    = static_cast<int>(aabbs.size());
    if(n > 0)
        init<<<cuda_tool::best_grid_dim(n, init), cuda_tool::best_block_dim(init), 0, stream>>>(
            n, m_queryId.view());
    Impl::calcMaxBVFromBox(aabbs, m_querySceneBox, stream);
    Impl::calcMCsFromBox(aabbs, m_querySceneBox, m_queryMtCode.view(), stream);
    cuda_tool::DeviceRadixSort(stream).SortPairs(m_queryMtCode.data(),
                                                 m_querySortedMtCode.data(),
                                                 m_queryId.data(),
                                                 m_querySortedId.data(),
                                                 n);
}

inline void InfoStacklessBVH::build(cuda_tool::CBufferView<AABB>   aabbs,
                                    cuda_tool::CBufferView<IndexT> BIDs,
                                    cuda_tool::CBufferView<IndexT> CIDs,
                                    cudaStream_t                   stream)
{
    m_aabbs = aabbs;
    m_BIDs  = BIDs;
    m_CIDs  = CIDs;
    UIPC_ASSERT(m_aabbs.size() == m_BIDs.size(),
                "AABB and BID size mismatch. aabbs=%zu, bids=%zu",
                m_aabbs.size(),
                m_BIDs.size());
    UIPC_ASSERT(m_aabbs.size() == m_CIDs.size(),
                "AABB and CID size mismatch. aabbs=%zu, cids=%zu",
                m_aabbs.size(),
                m_CIDs.size());
    m_impl.build(aabbs, BIDs, CIDs, stream);
}

inline void InfoStacklessBVH::build(cuda_tool::CBufferView<AABB> aabbs)
{
    m_aabbs = aabbs;
    m_BIDs  = {};
    m_CIDs  = {};
    m_impl.build(aabbs, {}, {});
}

inline bool InfoStacklessBVH::refit(cuda_tool::CBufferView<AABB>   aabbs,
                                    cuda_tool::CBufferView<IndexT> BIDs,
                                    cuda_tool::CBufferView<IndexT> CIDs,
                                    cudaStream_t                   stream)
{
    if(aabbs.size() != m_aabbs.size() || aabbs.size() != BIDs.size()
       || aabbs.size() != CIDs.size())
        return false;

    if(!m_impl.refit(aabbs, BIDs, CIDs, stream))
        return false;

    m_aabbs = aabbs;
    m_BIDs  = BIDs;
    m_CIDs  = CIDs;
    return true;
}

inline bool InfoStacklessBVH::prepare_query_result(QueryBuffer& qbuffer, int count)
{
    UIPC_ASSERT(count >= 0, "BVH query count must be non-negative, got %d", count);

    const bool retry = static_cast<size_t>(count) > qbuffer.m_pairs.size();
    if(retry)
    {
        const auto new_size = static_cast<size_t>(count * m_impl.config.reserve_ratio);
        qbuffer.m_pairs.reserve_discard(new_size);
        qbuffer.m_pairs.resize_discard(new_size);
    }
    qbuffer.m_size = count;
    return retry;
}

template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVH::launch_detect(cuda_tool::CBuffer2DView<IndexT> cmts,
                                            NodePred     np,
                                            LeafPred     lp,
                                            QueryBuffer& qbuffer)
{
    using namespace cuda_tool;
    BufferLaunch().fill(qbuffer.m_cpNum.view(), 0);

    if(m_aabbs.size() == 0)
        return;

    UIPC_ASSERT(m_aabbs.size() == m_BIDs.size(),
                "AABB and BID size mismatch. aabbs=%zu, bids=%zu",
                m_aabbs.size(),
                m_BIDs.size());
    UIPC_ASSERT(m_aabbs.size() == m_CIDs.size(),
                "AABB and CID size mismatch. aabbs=%zu, cids=%zu",
                m_aabbs.size(),
                m_CIDs.size());

    m_impl.stacklessSelf(np, lp, qbuffer.m_cpNum.view(), qbuffer.m_pairs.view());
}

// detect() with NodePred / LeafPred: wrapper passes pre-loaded bid/cid to NodePredInfo
template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVH::detect(cuda_tool::CBuffer2DView<IndexT> cmts,
                                     NodePred                         np,
                                     LeafPred                         lp,
                                     QueryBuffer&                     qbuffer)
{
    launch_detect(cmts, np, lp, qbuffer);
    int h_cp_num = qbuffer.m_cpNum;
    if(prepare_query_result(qbuffer, h_cp_num))
        launch_detect(cmts, np, lp, qbuffer);
}

template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVH::launch_query(cuda_tool::CBufferView<AABB> query_aabbs,
                                           cuda_tool::CBufferView<IndexT> query_BIDs,
                                           cuda_tool::CBufferView<IndexT> query_CIDs,
                                           cuda_tool::CBuffer2DView<IndexT> cmts,
                                           NodePred     np,
                                           LeafPred     lp,
                                           QueryBuffer& qbuffer,
                                           bool         rebuild_query,
                                           bool         reuse_query_order,
                                           cudaStream_t stream)
{
    using namespace cuda_tool;
    BufferLaunch(stream).fill(qbuffer.m_cpNum.view(), 0);

    if(m_aabbs.size() == 0 || query_aabbs.size() == 0)
        return;

    UIPC_ASSERT(query_aabbs.size() == query_BIDs.size(),
                "Query AABB and BID size mismatch. aabbs=%zu, bids=%zu",
                query_aabbs.size(),
                query_BIDs.size());
    UIPC_ASSERT(query_aabbs.size() == query_CIDs.size(),
                "Query AABB and CID size mismatch. aabbs=%zu, cids=%zu",
                query_aabbs.size(),
                query_CIDs.size());

    if(rebuild_query)
        qbuffer.build(query_aabbs, reuse_query_order, stream);
    m_impl.stacklessOther(np,
                          lp,
                          query_aabbs,
                          query_BIDs,
                          query_CIDs,
                          qbuffer.m_querySortedId.view(),
                          qbuffer.m_cpNum.view(),
                          qbuffer.m_pairs.view(),
                          stream);
}

// query() with NodePred / LeafPred: passes query BIDs/CIDs to stacklessOther for SMem pre-load
template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVH::query(cuda_tool::CBufferView<AABB>   query_aabbs,
                                    cuda_tool::CBufferView<IndexT> query_BIDs,
                                    cuda_tool::CBufferView<IndexT> query_CIDs,
                                    cuda_tool::CBuffer2DView<IndexT> cmts,
                                    NodePred                         np,
                                    LeafPred                         lp,
                                    QueryBuffer&                     qbuffer,
                                    bool reuse_query_order)
{
    launch_query(query_aabbs, query_BIDs, query_CIDs, cmts, np, lp, qbuffer, true, reuse_query_order);
    int h_cp_num = qbuffer.m_cpNum;
    if(prepare_query_result(qbuffer, h_cp_num))
        launch_query(query_aabbs, query_BIDs, query_CIDs, cmts, np, lp, qbuffer, false, reuse_query_order);
}

}  // namespace uipc::backend::cuda
