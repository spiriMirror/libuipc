#include <muda/atomic.h>

/*****************************************************************************************
 * Core Implementation
 *****************************************************************************************/

namespace uipc::backend::cuda
{
template <bool IsConst>
template <typename QueryType, typename IntersectF, typename CallbackF>
MUDA_DEVICE uint32_t LinearBVHViewerT<IsConst>::query(const QueryType& Q,
                                                      IntersectF Intersect,
                                                      uint32_t*  stack,
                                                      uint32_t   stack_num,
                                                      CallbackF Callback) const noexcept
{
    uint32_t* stack_ptr = stack;
    uint32_t* stack_end = stack + stack_num;
    *stack_ptr++        = 0;  // root node is always 0

    if(m_num_objects == 1)
    {
        if(Intersect(m_aabbs(0), Q))
        {
            Callback(m_nodes(0).object_idx);
            return 1;
        }
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


/*****************************************************************************************
 * API Implementation
 *****************************************************************************************/
namespace uipc::backend::cuda::detail
{
LinearBVHMortonIndex::LinearBVHMortonIndex(uint32_t m, uint32_t idx) noexcept
{
    m_morton_index = m;
    m_morton_index <<= 32;
    m_morton_index |= idx;
}

LinearBVHMortonIndex::operator uint64_t() const noexcept
{
    return m_morton_index;
}

bool operator==(const LinearBVHMortonIndex& lhs, const LinearBVHMortonIndex& rhs) noexcept
{
    return lhs.m_morton_index == rhs.m_morton_index;
}
}  // namespace uipc::backend::cuda::detail


namespace uipc::backend::cuda
{
template <bool IsConst>
LinearBVHViewerT<IsConst>::LinearBVHViewerT(const uint32_t       num_nodes,
                                            const uint32_t       num_objects,
                                            const LinearBVHNode* nodes,
                                            const LinearBVHAABB* aabbs)
    : m_num_nodes(num_nodes)
    , m_num_objects(num_objects)
    , m_nodes(nodes, num_nodes)
    , m_aabbs(aabbs, num_nodes)
{
}

template <bool IsConst>
template <bool _IsConst>
LinearBVHViewerT<IsConst>::LinearBVHViewerT(LinearBVHViewerT<_IsConst> viewer) noexcept
    : m_num_nodes(viewer.m_num_nodes)
    , m_num_objects(viewer.m_num_objects)
    , m_nodes(viewer.m_nodes)
    , m_aabbs(viewer.m_aabbs)
{
}

template <typename T>
void LinearBVH::resize(muda::Stream& s, muda::DeviceBuffer<T>& V, size_t size)
{
    if(size > V.capacity())
        muda::BufferLaunch(s).reserve(V, size * m_config.buffer_resize_factor);
    muda::BufferLaunch(s).resize(V, size);
}

template <bool IsConst>
MUDA_DEVICE bool LinearBVHViewerT<IsConst>::stack_overflow() const noexcept
{
    return m_stack_overflow;
}

template <bool IsConst>
MUDA_DEVICE void LinearBVHViewerT<IsConst>::check_index(const uint32_t idx) const noexcept
{
    MUDA_KERNEL_ASSERT(idx < m_num_objects,
                       "BVHViewer[%s:%s]: index out of range, idx=%u, num_objects=%u",
                       this->name(),
                       this->kernel_name(),
                       idx,
                       m_num_objects);
}

template <bool IsConst>
MUDA_DEVICE void LinearBVHViewerT<IsConst>::stack_overflow(uint32_t num_found,
                                                          uint32_t stack_num) const noexcept
{
    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        MUDA_KERNEL_WARN_WITH_LOCATION(
            "BVHViewer[%s:%s]: stack overflow, num_found=%u, stack_num=%u,"
            "the intersection count may be smaller than the ground truth, try enlarge the stack please.",
            this->name(),
            this->kernel_name(),
            num_found,
            stack_num);
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