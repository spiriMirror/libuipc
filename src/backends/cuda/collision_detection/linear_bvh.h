/*****************************************************************//**
 * \file   linear_bvh.h
 * \brief  The LinearBVH class and its viewer class.
 * 
 * NOTE: We make the LinerBVH header-only to make unit testing easier.
 * Otherwise, if using dllexport/dllimport, cuda will throw "InvalidDeviceFunction" error,
 * On Windows, MSVC 14.39.33519, with CUDA Version >= 12.4.
 * 
 * \author MuGdxy
 * \date   September 2024
 *********************************************************************/

#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <Eigen/Geometry>
#include <collision_detection/aabb.h>

namespace uipc::backend::cuda
{
namespace detail
{
    struct LinearBVHMortonIndex
    {
        MUDA_GENERIC LinearBVHMortonIndex(uint32_t m, uint32_t idx) noexcept;

        MUDA_GENERIC LinearBVHMortonIndex() noexcept = default;

        MUDA_GENERIC operator uint64_t() const noexcept;

      private:
        friend MUDA_GENERIC bool operator==(const LinearBVHMortonIndex& lhs,
                                            const LinearBVHMortonIndex& rhs) noexcept;
        uint64_t    m_morton_index = 0;
    };

    MUDA_GENERIC bool operator==(const LinearBVHMortonIndex& lhs,
                                 const LinearBVHMortonIndex& rhs) noexcept;
}  // namespace detail

class LinearBVHNode
{
  public:
    uint32_t parent_idx = 0xFFFFFFFF;  // parent node
    uint32_t left_idx   = 0xFFFFFFFF;  // index of left  child node
    uint32_t right_idx  = 0xFFFFFFFF;  // index of right child node
    uint32_t object_idx = 0xFFFFFFFF;  // == 0xFFFFFFFF if internal node.

    MUDA_GENERIC bool is_leaf() const noexcept;
    MUDA_GENERIC bool is_internal() const noexcept;
    MUDA_GENERIC bool is_top() const noexcept;
};

using LinearBVHAABB = AABB;

class LinearBVH;

class LinearBVHViewer : public muda::ViewerBase<true>
{
    MUDA_VIEWER_COMMON_NAME(LinearBVHViewer);

    using Base = muda::ViewerBase<true>;

    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    friend class LinearBVH;

    constexpr static uint32_t DEFAULT_STACK_SIZE = 64;

  public:
    struct DefaultQueryCallback
    {
        MUDA_GENERIC void operator()(uint32_t obj_idx) const noexcept {}
    };

    MUDA_GENERIC LinearBVHViewer(const uint32_t       num_nodes,
                                 const uint32_t       num_objects,
                                 const LinearBVHNode* nodes,
                                 const LinearBVHAABB* aabbs);

    MUDA_GENERIC LinearBVHViewer(const LinearBVHViewer&)            = default;
    MUDA_GENERIC LinearBVHViewer(LinearBVHViewer&&)                 = default;
    MUDA_GENERIC LinearBVHViewer& operator=(const LinearBVHViewer&) = default;
    MUDA_GENERIC LinearBVHViewer& operator=(LinearBVHViewer&&)      = default;

    MUDA_GENERIC auto num_nodes() const noexcept { return m_num_nodes; }
    MUDA_GENERIC auto num_objects() const noexcept { return m_num_objects; }

    /**
     * @brief query AABBs that intersect with the given point m_q.
     * 
     * @param m_q query point
     * @param callback callback function that is called when an AABBs is found (may be called multiple times)
     * 
     * @return the number of found AABBs
     */
    template <uint32_t StackNum = DEFAULT_STACK_SIZE, std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_DEVICE uint32_t query(const Vector3& q,
                               CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        uint32_t stack[StackNum];
        return this->query(q, stack, StackNum, callback);
    }

    template <std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_DEVICE uint32_t query(const Vector3& q,
                               uint32_t*      stack,
                               uint32_t       stack_num,
                               CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        return this->query(
            q,
            [](const LinearBVHAABB& node, const Vector3& q)
            { return node.contains(q); },
            stack,
            stack_num,
            callback);
    }

    /**
     * @brief query AABBs that intersect with the given AABBs m_q.
     * 
     * @param m_q query AABBs
     * @param callback callback function that is called when an AABBs is found (may be called multiple times)
     * 
     * @return the number of found AABBs
     */
    template <uint32_t StackNum = DEFAULT_STACK_SIZE, std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_DEVICE uint32_t query(const LinearBVHAABB& aabb,
                               CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        uint32_t stack[StackNum];
        return this->query(aabb, stack, StackNum, callback);
    }

    template <std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_DEVICE uint32_t query(const LinearBVHAABB& aabb,
                               uint32_t*            stack,
                               uint32_t             stack_num,
                               CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        return this->query(
            aabb,
            [](const LinearBVHAABB& node, const LinearBVHAABB& Q)
            { return node.intersects(Q); },
            stack,
            stack_num,
            callback);
    }

    /**
     * @brief check if the stack overflow occurs during the query.
     */
    MUDA_DEVICE bool stack_overflow() const noexcept;

  private:
    uint32_t m_num_nodes;    // (# of internal node) + (# of leaves), 2N+1
    uint32_t m_num_objects;  // (# of leaves), the same as the number of objects

    muda::CDense1D<LinearBVHAABB> m_aabbs;
    muda::CDense1D<LinearBVHNode> m_nodes;

    mutable int m_stack_overflow = false;

    MUDA_DEVICE void check_index(const uint32_t idx) const noexcept;

    MUDA_DEVICE void stack_overflow(uint32_t num_found, uint32_t stack_num) const noexcept;

    template <typename QueryType, typename IntersectF, typename CallbackF>
    MUDA_DEVICE uint32_t query(const QueryType& Q,
                               IntersectF       Intersect,
                               uint32_t*        stack,
                               uint32_t         stack_num,
                               CallbackF        Callback) const noexcept;
};

/**
 * @brief Configuration for LinearBVH Tree.
 */
class LinearBVHConfig
{
  public:
    Float buffer_resize_factor = 1.5;
};

/**
 * @brief LinearBVH Tree class.
 */
class LinearBVH
{
    friend class LinearBVHVisitor;

  public:
    using MortonIndex = detail::LinearBVHMortonIndex;

    LinearBVH(const LinearBVHConfig& config = {}) noexcept;
    LinearBVH(const LinearBVH&)            = default;
    LinearBVH(LinearBVH&&)                 = default;
    LinearBVH& operator=(const LinearBVH&) = default;
    LinearBVH& operator=(LinearBVH&&)      = default;

    /**
     * @brief Construct LinearBVH Tree of given AABBs.
     * 
     * @param aabb The array of AABBs
     * @param s The stream to execute the construction
     */
    void build(muda::CBufferView<LinearBVHAABB> aabbs,
               muda::Stream&                    s = muda::Stream::Default());

    /**
     * @brief Keep the constructed LinearBVH Tree and update the AABBs.
     * 
     * The `update()` performs better than `build()` because it reuses the constructed LinearBVH Tree. 
     * However, if the AABBs are significantly changed (bad time coherence), you may get a Tree with bad query performance.
     * So, if the AABBs are significantly changed, you should call `build()` instead of `update()` to reconstruct the LinearBVH Tree.
     * 
     * @param aabbs The array of AABBs
     * @param s The stream to execute the update
     */
    void update(muda::CBufferView<LinearBVHAABB> aabbs,
                muda::Stream&                    s = muda::Stream::Default());

    /**
     * @brief Get a query handler for the constructed LinearBVH tree.
     */
    LinearBVHViewer viewer() const noexcept;

  private:
    template <typename T>
    void resize(muda::Stream& s, muda::DeviceBuffer<T>& V, size_t size);

    muda::DeviceBuffer<LinearBVHAABB> m_aabbs;
    muda::DeviceBuffer<uint32_t>      m_mortons;
    muda::DeviceBuffer<uint32_t>      m_sorted_mortons;
    muda::DeviceBuffer<uint32_t>      m_indices;
    muda::DeviceBuffer<uint32_t>      m_new_to_old;
    muda::DeviceBuffer<MortonIndex>   m_morton_idx;
    muda::DeviceBuffer<int>           m_flags;
    muda::DeviceBuffer<LinearBVHNode> m_nodes;
    muda::DeviceVar<LinearBVHAABB>    m_max_aabb;

    LinearBVHConfig m_config;

    void build_internal_aabbs(muda::Stream& s);
};

/**
 * @brief Visitor class for LinearBVH, which provides the advanced information of the constructed LinearBVH tree.
 */
class LinearBVHVisitor
{
  public:
    LinearBVHVisitor(LinearBVH& bvh) noexcept;
    LinearBVHVisitor(const LinearBVHVisitor&)            = default;
    LinearBVHVisitor(LinearBVHVisitor&&)                 = default;
    LinearBVHVisitor& operator=(const LinearBVHVisitor&) = default;

    muda::CBufferView<LinearBVHNode> nodes() const noexcept;
    muda::CBufferView<LinearBVHNode> object_nodes() const noexcept;
    muda::CBufferView<LinearBVHAABB> aabbs() const noexcept;
    muda::CVarView<LinearBVHAABB>    top_aabb() const noexcept;

  private:
    LinearBVH& m_bvh;
};
}  // namespace uipc::backend::cuda

#include "details/linear_bvh.inl"
