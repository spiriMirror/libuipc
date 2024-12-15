#include <muda/atomic.h>
#include <muda/cub/device/device_scan.h>
#include <muda/cub/device/device_reduce.h>
#include <muda/cub/device/device_radix_sort.h>

#include <cub/util_ptx.cuh>
#include <cuda/atomic>
#include <muda/atomic.h>
#include <muda/ext/eigen/atomic.h>
#include <thrust/detail/minmax.h>

#include <uipc/common/log.h>

/*****************************************************************************************
 * Viewer Core Implementation
 *****************************************************************************************/

namespace uipc::backend::cuda
{
template <typename QueryType, typename IntersectF, typename CallbackF>
MUDA_DEVICE uint32_t LinearBVHViewer::query(const QueryType& Q,
                                            IntersectF       Intersect,
                                            uint32_t*        stack,
                                            uint32_t         stack_num,
                                            CallbackF Callback) const noexcept
{
    uint32_t* stack_ptr = stack;
    uint32_t* stack_end = stack + stack_num;
    *stack_ptr++        = 0;  // root node is always 0

    if(m_num_objects == 0)
        return 0;

    if(m_num_objects == 1)
    {
        if(Intersect(m_aabbs(0), Q))
        {
            Callback(m_nodes(0).object_idx);
            return 1;
        }
        return 0;
    }

    uint32_t num_found = 0;
    do
    {
        const uint32_t node  = *--stack_ptr;
        const uint32_t L_idx = m_nodes(node).left_idx;
        const uint32_t R_idx = m_nodes(node).right_idx;

        if(Intersect(m_aabbs(L_idx), Q))
        {
            const auto obj_idx = m_nodes(L_idx).object_idx;
            if(obj_idx != 0xFFFFFFFF)
            {
                check_index(obj_idx);
                Callback(obj_idx);
                ++num_found;
            }
            else  // the node is not a leaf.
            {
                *stack_ptr++ = L_idx;
            }
        }
        if(Intersect(m_aabbs(R_idx), Q))
        {
            const auto obj_idx = m_nodes(R_idx).object_idx;
            if(obj_idx != 0xFFFFFFFF)
            {
                check_index(obj_idx);
                Callback(obj_idx);
                ++num_found;
            }
            else  // the node is not a leaf.
            {
                *stack_ptr++ = R_idx;
            }
        }
        if(stack_ptr >= stack_end)
        {
            stack_overflow(num_found, stack_num);
            break;
        }
    } while(stack < stack_ptr);
    return num_found;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
MUDA_INLINE LinearBVHViewer::LinearBVHViewer(const uint32_t       num_nodes,
                                             const uint32_t       num_objects,
                                             const LinearBVHNode* nodes,
                                             const LinearBVHAABB* aabbs)
    : m_num_nodes(num_nodes)
    , m_num_objects(num_objects)
    , m_nodes(nodes, num_nodes)
    , m_aabbs(aabbs, num_nodes)
{
}

template <typename T>
void LinearBVH::resize(muda::Stream& s, muda::DeviceBuffer<T>& V, size_t size)
{
    if(size > V.capacity())
        muda::BufferLaunch(s).reserve(V, size * m_config.buffer_resize_factor);
    muda::BufferLaunch(s).resize(V, size);
}

MUDA_INLINE MUDA_DEVICE bool LinearBVHViewer::stack_overflow() const noexcept
{
    return m_stack_overflow;
}

MUDA_INLINE MUDA_DEVICE void LinearBVHViewer::check_index(const uint32_t idx) const noexcept
{
    MUDA_KERNEL_ASSERT(idx < m_num_objects,
                       "BVHViewer[%s:%s]: index out of range, idx=%u, num_objects=%u. %s(%d)",
                       this->name(),
                       this->kernel_name(),
                       idx,
                       m_num_objects,
                       this->kernel_file(),
                       this->kernel_line());
}

MUDA_INLINE MUDA_DEVICE void LinearBVHViewer::stack_overflow(uint32_t num_found,
                                                             uint32_t stack_num) const noexcept
{
    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        MUDA_KERNEL_WARN_WITH_LOCATION(
            "BVHViewer[%s:%s]: stack overflow, num_found=%u, stack_num=%u,"
            "the intersection count may be smaller than the ground truth, try enlarge the stack please. %s(%d)",
            this->name(),
            this->kernel_name(),
            num_found,
            stack_num,
            this->kernel_file(),
            this->kernel_line());
    }

    m_stack_overflow = 1;
}

MUDA_INLINE bool LinearBVHNode::is_leaf() const noexcept
{
    return object_idx != 0xFFFFFFFF;
}

MUDA_INLINE bool LinearBVHNode::is_top() const noexcept
{
    return parent_idx == 0xFFFFFFFF;
}

MUDA_INLINE bool LinearBVHNode::is_internal() const noexcept
{
    return object_idx == 0xFFFFFFFF;
}
}  // namespace uipc::backend::cuda


/*********************************************************************************
* LinearBVH Core Implementation
*********************************************************************************/

namespace uipc::backend::cuda::detail
{
MUDA_INLINE MUDA_DEVICE int common_upper_bits(const unsigned long long int lhs,
                                              const unsigned long long int rhs) noexcept
{
#ifdef __CUDA_ARCH__
    return ::__clzll(lhs ^ rhs);
#else
    MUDA_ASSERT(false, "common_upper_bits is not supported on the host.");
    return -1;
#endif
}

MUDA_INLINE MUDA_GENERIC std::uint32_t expand_bits(std::uint32_t v) noexcept
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

MUDA_INLINE MUDA_GENERIC std::uint32_t morton_code(Vector3 xyz) noexcept
{
    xyz = xyz.cwiseMin(1.0).cwiseMax(0.0);
    const std::uint32_t xx = expand_bits(static_cast<std::uint32_t>(xyz.x() * 1024.0));
    const std::uint32_t yy = expand_bits(static_cast<std::uint32_t>(xyz.y() * 1024.0));
    const std::uint32_t zz = expand_bits(static_cast<std::uint32_t>(xyz.z() * 1024.0));
    return xx * 4 + yy * 2 + zz;
}

MUDA_INLINE MUDA_DEVICE uint2 determine_range(muda::Dense1D<LinearBVHMortonIndex> node_code,
                                              const uint32_t num_leaves,
                                              uint32_t       idx)
{
    if(idx == 0)
    {
        return make_uint2(0, num_leaves - 1);
    }

    // determine direction of the range
    const auto self_code = node_code(idx);
    const int  L_delta   = common_upper_bits(self_code, node_code(idx - 1));
    const int  R_delta   = common_upper_bits(self_code, node_code(idx + 1));
    const int  d         = (R_delta > L_delta) ? 1 : -1;

    // Compute upper bound for the length of the range

    const int delta_min = thrust::min(L_delta, R_delta);
    int       l_max     = 2;
    int       delta     = -1;
    int       i_tmp     = idx + d * l_max;
    if(0 <= i_tmp && i_tmp < num_leaves)
    {
        delta = common_upper_bits(self_code, node_code(i_tmp));
    }
    while(delta > delta_min)
    {
        l_max <<= 1;
        i_tmp = idx + d * l_max;
        delta = -1;
        if(0 <= i_tmp && i_tmp < num_leaves)
        {
            delta = common_upper_bits(self_code, node_code(i_tmp));
        }
    }

    // Find the other end by binary search
    int l = 0;
    int t = l_max >> 1;
    while(t > 0)
    {
        i_tmp = idx + (l + t) * d;
        delta = -1;
        if(0 <= i_tmp && i_tmp < num_leaves)
        {
            delta = common_upper_bits(self_code, node_code(i_tmp));
        }
        if(delta > delta_min)
        {
            l += t;
        }
        t >>= 1;
    }
    uint32_t jdx = idx + l * d;
    if(d < 0)
    {
        thrust::swap(idx, jdx);  // make it sure that idx < jdx
    }
    return make_uint2(idx, jdx);
}


MUDA_INLINE MUDA_DEVICE uint32_t find_split(muda::Dense1D<LinearBVHMortonIndex> node_code,
                                            const uint32_t num_leaves,
                                            const uint32_t first,
                                            const uint32_t last) noexcept
{
    const auto first_code = node_code(first);
    const auto last_code  = node_code(last);

    if(first_code == last_code)
    {
        return (first + last) >> 1;
    }
    const int delta_node = common_upper_bits(first_code, last_code);

    // binary search...
    int split  = first;
    int stride = last - first;
    do
    {
        stride           = (stride + 1) >> 1;
        const int middle = split + stride;
        if(middle < last)
        {
            const int delta = common_upper_bits(first_code, node_code(middle));
            if(delta > delta_node)
            {
                split = middle;
            }
        }
    } while(stride > 1);

    return split;
}


MUDA_INLINE void build_internal_aabbs(size_t num_objects,
                                      muda::CBufferView<LinearBVHNode> nodes,
                                      muda::BufferView<LinearBVHAABB> sorted_aabbs,
                                      muda::BufferView<int> flags,
                                      muda::Stream&         s)
{
    using namespace muda;

    auto num_internal_nodes = num_objects - 1;
    auto leaf_start         = num_internal_nodes;

    auto internal_aabbs = sorted_aabbs.subview(0, num_internal_nodes);
    on(s)
        .next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(num_objects,
               [nodes = nodes.cviewer().name("nodes"),
                aabbs = sorted_aabbs.viewer().name("aabbs"),
                flags = flags.viewer().name("flags"),
                leaf_start] __device__(int I) mutable
               {
                   auto leaf_idx = I + leaf_start;
                   auto parent   = nodes(leaf_idx).parent_idx;

                   while(parent != 0xFFFFFFFF)  // means idx == 0
                   {
                       const int old = muda::atomic_add(&flags(parent), 1);

                       // the memory fence is necessary to disable reordering of the memory access.
                       // we need to ensure that this thread can get the updated value of AABB.
                       ::cuda::atomic_thread_fence(::cuda::memory_order_acquire,
                                                   ::cuda::thread_scope_system);

                       if(old == 0)
                       {
                           // this is the first thread entered here.
                           // wait the other thread from the other child node.
                           return;
                       }
                       MUDA_KERNEL_ASSERT(old == 1, "old=%d", old);
                       //here, the flag has already been 1. it means that this
                       //thread is the 2nd thread. merge AABB of both childlen.

                       const auto lidx = nodes(parent).left_idx;
                       const auto ridx = nodes(parent).right_idx;
                       auto&      lbox = aabbs(lidx);
                       auto&      rbox = aabbs(ridx);

                       // to avoid cache coherency problem, we must use atomic operation.
                       auto atomic_fetch = [](LinearBVHAABB& aabb) -> LinearBVHAABB
                       {
                           Vector3       zero  = Vector3::Zero();
                           LinearBVHAABB aabb_ = aabb;

                           // without atomic_thread_fence, this loop may be infinite.
                           while(aabb_.isEmpty())
                           {
                               Vector3 min_ = eigen::atomic_add(aabb.min(), zero);
                               Vector3 max_ = eigen::atomic_add(aabb.max(), zero);
                               aabb_ = LinearBVHAABB{min_, max_};
                           };

                           return aabb_;
                       };

                       auto L = atomic_fetch(lbox);
                       auto R = atomic_fetch(rbox);

                       aabbs(parent) = L.merged(R);

                       // look the next parent...
                       parent = nodes(parent).parent_idx;
                   }
               });
}


}  // namespace uipc::backend::cuda::detail

namespace uipc::backend::cuda
{
MUDA_INLINE void LinearBVH::build(muda::CBufferView<LinearBVHAABB> aabbs, muda::Stream& s)
{
    using namespace muda;

    if(aabbs.size() == 0)
        return;

    const uint32_t num_objects        = aabbs.size();
    const uint32_t num_internal_nodes = num_objects - 1;
    const uint32_t leaf_start         = num_internal_nodes;
    const uint32_t num_nodes          = num_objects * 2 - 1;

    LinearBVHAABB default_aabb;
    resize(s, m_aabbs, num_nodes);
    BufferLaunch(s).fill(m_aabbs.view(), default_aabb);

    resize(s, m_sorted_mortons, num_objects);
    resize(s, m_sorted_mortons, num_objects);

    resize(s, m_indices, num_objects);
    resize(s, m_new_to_old, num_objects);

    resize(s, m_mortons, num_objects);
    resize(s, m_morton_idx, num_objects);

    LinearBVHNode default_node;
    m_nodes.resize(num_nodes, default_node);
    resize(s, m_nodes, num_nodes);

    // 1) get max aabb
    DeviceReduce(s).Reduce(
        aabbs.data(),
        m_max_aabb.data(),
        aabbs.size(),
        [] CUB_RUNTIME_FUNCTION(const LinearBVHAABB& a, const LinearBVHAABB& b) noexcept -> LinearBVHAABB
        { return a.merged(b); },
        default_aabb);

    // 2) calculate m_morton_index code
    on(s)
        .next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(num_objects,
               [max_aabb = m_max_aabb.viewer().name("max_aabb"),
                aabbs    = aabbs.viewer().name("filled_aabbs"),
                mortons = m_mortons.viewer().name("mortons")] __device__(int i) mutable
               {
                   auto&   aabb = aabbs(i);
                   Vector3 p    = aabb.center();

                   MUDA_ASSERT(aabbs(i).volume() >= 0,
                               "Invalid AABB(%d), Max(%f,%f,%f) < Min(%f,%f,%f)",

                               i,
                               aabb.max().x(),
                               aabb.max().y(),
                               aabb.max().z(),
                               aabb.min().x(),
                               aabb.min().y(),
                               aabb.min().z());

                   p -= max_aabb->min();
                   p.array() /= max_aabb->sizes().array();
                   mortons(i) = detail::morton_code(p);
               });

    // 3) sort m_morton_index code
    on(s)
        .next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(m_indices.size(),
               [indices = m_indices.viewer()] __device__(int i) mutable
               { indices(i) = i; });

    // 4) sort m_morton_index code
    DeviceRadixSort(s).SortPairs(m_mortons.data(),
                                 m_sorted_mortons.data(),
                                 m_indices.data(),
                                 m_new_to_old.data(),
                                 num_objects);

    // 5) expand m_morton_index code to 64bit, the last 32bit is the index
    on(s)
        .next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(m_mortons.size(),
               [morton64s = m_morton_idx.viewer().name("morton64s"),
                mortons   = m_sorted_mortons.viewer().name("mortons"),
                indices = m_new_to_old.viewer().name("indices")] __device__(int i) mutable
               {
                   MortonIndex morton{mortons(i), indices(i)};
                   morton64s(i) = morton;
               });

    // 6) setup leaf nodes
    auto leaf_aabbs = m_aabbs.view(leaf_start);  // offset = leaf_start
    auto leaf_nodes = m_nodes.view(leaf_start);  // offset = leaf_start
    on(s)
        .next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(num_objects,
               [leaf_nodes = leaf_nodes.viewer().name("leaf_nodes"),
                indices    = m_new_to_old.viewer().name("indices"),
                aabbs      = aabbs.viewer().name("aabbs"),
                sorted_aabbs = leaf_aabbs.viewer().name("sorted_aabbs")] __device__(int i) mutable
               {
                   LinearBVHNode node;
                   node.parent_idx = 0xFFFFFFFF;
                   node.left_idx   = 0xFFFFFFFF;
                   node.right_idx  = 0xFFFFFFFF;
                   node.object_idx = indices(i);
                   leaf_nodes(i)   = node;
                   sorted_aabbs(i) = aabbs(node.object_idx);
               });

    // 7) construct internal nodes
    on(s)
        .next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(num_internal_nodes,
               [nodes      = m_nodes.viewer().name("nodes"),
                morton_idx = m_morton_idx.viewer().name("morton_idx"),
                num_objects] __device__(int idx) mutable
               {
                   nodes(idx).object_idx = 0xFFFFFFFF;  //  internal nodes

                   const uint2 ij = detail::determine_range(morton_idx, num_objects, idx);
                   const int gamma =
                       detail::find_split(morton_idx, num_objects, ij.x, ij.y);

                   nodes(idx).left_idx  = gamma;
                   nodes(idx).right_idx = gamma + 1;
                   if(thrust::min(ij.x, ij.y) == gamma)
                   {
                       nodes(idx).left_idx += num_objects - 1;
                   }
                   if(thrust::max(ij.x, ij.y) == gamma + 1)
                   {
                       nodes(idx).right_idx += num_objects - 1;
                   }
                   nodes(nodes(idx).left_idx).parent_idx  = idx;
                   nodes(nodes(idx).right_idx).parent_idx = idx;
               });

    // 8) calculate the AABB of internal nodes
    build_internal_aabbs(s);
}

MUDA_INLINE void LinearBVH::update(muda::CBufferView<LinearBVHAABB> aabbs, muda::Stream& s)
{
    using namespace muda;

    UIPC_ASSERT(aabbs.size() == m_indices.size(),
                "To update the tree, the number of input AABBs({}) must be the same as the original number of objects ({}) in the BVH.",
                aabbs.size(),
                m_indices.size());

    // 1) update leaf aabbs
    auto leaf_aabbs = m_aabbs.view(m_nodes.size() - aabbs.size());
    on().next<ParallelFor>()
        .file_line(__FILE__, __LINE__)
        .apply(aabbs.size(),
               [indices = m_new_to_old.viewer().name("indices"),
                aabbs   = aabbs.viewer().name("aabbs"),
                leaf_aabbs = leaf_aabbs.viewer().name("sorted_aabbs")] __device__(int i) mutable
               { leaf_aabbs(i) = aabbs(indices(i)); });

    // 2) update internal aabbs
    build_internal_aabbs(s);
}

MUDA_INLINE void LinearBVH::build_internal_aabbs(muda::Stream& s)
{
    using namespace muda;
    auto num_internal_nodes = m_nodes.size() - m_indices.size();
    resize(s, m_flags, num_internal_nodes);
    BufferLaunch(s).fill(m_flags.view(), 0);
    detail::build_internal_aabbs(m_indices.size(), m_nodes, m_aabbs, m_flags, s);
}
}  // namespace uipc::backend::cuda


/*****************************************************************************************
 * API Implementation
 *****************************************************************************************/
namespace uipc::backend::cuda::detail
{
MUDA_INLINE LinearBVHMortonIndex::LinearBVHMortonIndex(uint32_t m, uint32_t idx) noexcept
{
    m_morton_index = m;
    m_morton_index <<= 32;
    m_morton_index |= idx;
}

MUDA_INLINE LinearBVHMortonIndex::operator uint64_t() const noexcept
{
    return m_morton_index;
}

MUDA_INLINE bool operator==(const LinearBVHMortonIndex& lhs,
                            const LinearBVHMortonIndex& rhs) noexcept
{
    return lhs.m_morton_index == rhs.m_morton_index;
}
}  // namespace uipc::backend::cuda::detail

namespace uipc::backend::cuda
{
MUDA_INLINE LinearBVH::LinearBVH(const LinearBVHConfig& config) noexcept {}

MUDA_INLINE LinearBVHViewer LinearBVH::viewer() const noexcept
{
    return LinearBVHViewer{(uint32_t)m_nodes.size(),
                           (uint32_t)m_mortons.size(),
                           m_nodes.data(),
                           m_aabbs.data()};
}

MUDA_INLINE LinearBVHVisitor::LinearBVHVisitor(LinearBVH& bvh) noexcept
    : m_bvh(bvh)
{
}

MUDA_INLINE muda::CBufferView<LinearBVHNode> LinearBVHVisitor::nodes() const noexcept
{
    return m_bvh.m_nodes;
}

MUDA_INLINE muda::CBufferView<LinearBVHNode> LinearBVHVisitor::object_nodes() const noexcept
{
    auto object_count = m_bvh.m_indices.size();
    auto node_offset  = m_bvh.m_nodes.size() - object_count;
    return m_bvh.m_nodes.view(node_offset);
}

MUDA_INLINE muda::CBufferView<LinearBVHAABB> LinearBVHVisitor::aabbs() const noexcept
{
    return m_bvh.m_aabbs;
}

MUDA_INLINE muda::CVarView<LinearBVHAABB> LinearBVHVisitor::top_aabb() const noexcept
{
    return m_bvh.m_max_aabb;
}
}  // namespace uipc::backend::cuda
