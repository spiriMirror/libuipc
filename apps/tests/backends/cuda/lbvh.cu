#include <muda/ext/eigen/eigen_core_cxx20.h>  // to use Eigen in CUDA
#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/common/type_define.h>
#include <Eigen/Geometry>
#include <muda/buffer/device_buffer.h>
#include <muda/cub/device/device_reduce.h>
#include <muda/cub/device/device_radix_sort.h>
#include <uipc/geometry.h>
#include <uipc/common/enumerate.h>
#include <muda/atomic.h>
#include <muda/cub/device/device_scan.h>

namespace uipc::test::backend::cuda
{
namespace detail
{
    MUDA_DEVICE MUDA_INLINE int common_upper_bits(const unsigned int lhs,
                                                  const unsigned int rhs) noexcept
    {
        return ::__clz(lhs ^ rhs);
    }
    MUDA_DEVICE MUDA_INLINE int common_upper_bits(const unsigned long long int lhs,
                                                  const unsigned long long int rhs) noexcept
    {
        return ::__clzll(lhs ^ rhs);
    }

    MUDA_GENERIC MUDA_INLINE std::uint32_t expand_bits(std::uint32_t v) noexcept
    {
        v = (v * 0x00010001u) & 0xFF0000FFu;
        v = (v * 0x00000101u) & 0x0F00F00Fu;
        v = (v * 0x00000011u) & 0xC30C30C3u;
        v = (v * 0x00000005u) & 0x49249249u;
        return v;
    }

    MUDA_GENERIC MUDA_INLINE std::uint32_t morton_code(Vector3 xyz) noexcept
    {
        xyz = xyz.cwiseMin(1.0).cwiseMax(0.0);
        const std::uint32_t xx =
            expand_bits(static_cast<std::uint32_t>(xyz.x() * 1024.0));
        const std::uint32_t yy =
            expand_bits(static_cast<std::uint32_t>(xyz.y() * 1024.0));
        const std::uint32_t zz =
            expand_bits(static_cast<std::uint32_t>(xyz.z() * 1024.0));
        return xx * 4 + yy * 2 + zz;
    }

    struct LBVHNode
    {
        uint32_t parent_idx = 0xFFFFFFFF;  // parent node
        uint32_t left_idx   = 0xFFFFFFFF;  // index of left  child node
        uint32_t right_idx  = 0xFFFFFFFF;  // index of right child node
        uint32_t object_idx = 0xFFFFFFFF;  // == 0xFFFFFFFF if internal node.
    };

    struct LBVHMortonIndex
    {
        MUDA_GENERIC LBVHMortonIndex(uint32_t m, uint32_t idx) noexcept
        {
            morton = m;
            morton <<= 32;
            morton |= idx;
        }

        MUDA_GENERIC LBVHMortonIndex() noexcept = default;

        uint64_t morton = ~0ull;
        // uint32_t idx;
    };

    MUDA_GENERIC bool operator==(const LBVHMortonIndex& lhs, const LBVHMortonIndex& rhs) noexcept
    {
        return lhs.morton == rhs.morton;
        //&&lhs.idx == rhs.idx;
    }

    MUDA_DEVICE uint2 determine_range(muda::Dense1D<LBVHMortonIndex> node_code,
                                      const uint32_t                 num_leaves,
                                      uint32_t                       idx)
    {
        if(idx == 0)
        {
            return make_uint2(0, num_leaves - 1);
        }

        // determine direction of the range
        const auto self_code = node_code(idx);
        const int  L_delta =
            common_upper_bits(self_code.morton, node_code(idx - 1).morton);
        const int R_delta =
            common_upper_bits(self_code.morton, node_code(idx + 1).morton);
        const int d = (R_delta > L_delta) ? 1 : -1;

        // Compute upper bound for the length of the range

        const int delta_min = thrust::min(L_delta, R_delta);
        int       l_max     = 2;
        int       delta     = -1;
        int       i_tmp     = idx + d * l_max;
        if(0 <= i_tmp && i_tmp < num_leaves)
        {
            delta = common_upper_bits(self_code.morton, node_code(i_tmp).morton);
        }
        while(delta > delta_min)
        {
            l_max <<= 1;
            i_tmp = idx + d * l_max;
            delta = -1;
            if(0 <= i_tmp && i_tmp < num_leaves)
            {
                delta = common_upper_bits(self_code.morton, node_code(i_tmp).morton);
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
                delta = common_upper_bits(self_code.morton, node_code(i_tmp).morton);
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


    MUDA_DEVICE uint32_t find_split(muda::Dense1D<LBVHMortonIndex> node_code,
                                    const uint32_t                 num_leaves,
                                    const uint32_t                 first,
                                    const uint32_t last) noexcept
    {
        const auto first_code = node_code(first);
        const auto last_code  = node_code(last);
        if(first_code == last_code)
        {
            return (first + last) >> 1;
        }
        const int delta_node = common_upper_bits(first_code.morton, last_code.morton);

        // binary search...
        int split  = first;
        int stride = last - first;
        do
        {
            stride           = (stride + 1) >> 1;
            const int middle = split + stride;
            if(middle < last)
            {
                const int delta =
                    common_upper_bits(first_code.morton, node_code(middle).morton);
                if(delta > delta_node)
                {
                    split = middle;
                }
            }
        } while(stride > 1);

        return split;
    }
}  // namespace detail

class LBVH;

template <bool IsConst>
class LBVHViewerT : muda::ViewerBase<IsConst>
{
    MUDA_VIEWER_COMMON_NAME(LBVHViewerT);

    using Base = muda::ViewerBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    friend class LBVH;
    using Node = detail::LBVHNode;

  public:
    using ConstViewer    = LBVHViewerT<true>;
    using NonConstViewer = LBVHViewerT<false>;
    using ThisViewer     = LBVHViewerT<IsConst>;
    using AABB           = Eigen::AlignedBox<Float, 3>;

    struct DefaultQueryCallback
    {
        MUDA_GENERIC void operator()(uint32_t obj_idx) const noexcept {}
    };

    MUDA_GENERIC LBVHViewerT(const uint32_t      num_nodes,
                             const uint32_t      num_objects,
                             auto_const_t<Node>* nodes,
                             auto_const_t<AABB>* aabbs)
        : m_num_nodes(num_nodes)
        , m_num_objects(num_objects)
        , m_nodes(nodes)
        , m_aabbs(aabbs)
    {
        MUDA_KERNEL_ASSERT(m_nodes && m_aabbs,
                           "BVHViewerBase[%s:%s]: nullptr is passed,"
                           "nodes=%p,"
                           "aabbs=%p,"
                           "objects=%p\n",
                           this->name(),
                           this->kernel_name(),
                           m_nodes,
                           m_aabbs);
    }

    MUDA_GENERIC auto as_const() const noexcept
    {
        return ConstViewer{m_num_nodes, m_num_objects, m_nodes, m_aabbs};
    }

    MUDA_GENERIC operator ConstViewer() const noexcept { return as_const(); }

    MUDA_GENERIC auto num_nodes() const noexcept { return m_num_nodes; }
    MUDA_GENERIC auto num_objects() const noexcept { return m_num_objects; }

    /**
     * @brief query AABBs that intersect with the given point q.
     * 
     * @param q query point
     * @param callback callback function that is called when an AABB is found (may be called multiple times)
     * 
     * @return the number of found AABBs
     */
    template <uint32_t StackNum = 64, std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_GENERIC uint32_t query(const Vector3& q,
                                CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        uint32_t stack[StackNum];
        return this->query(q, stack, StackNum, callback);
    }

    template <std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_GENERIC uint32_t query(const Vector3& q,
                                uint32_t*      stack,
                                uint32_t       stack_num,
                                CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        return this->query(
            q,
            [](const AABB& aabb, const Vector3& q) { return aabb.contains(q); },
            stack,
            stack_num,
            callback);
    }

    /**
     * @brief query AABBs that intersect with the given AABB q.
     * 
     * @param q query AABB
     * @param callback callback function that is called when an AABB is found (may be called multiple times)
     * 
     * @return the number of found AABBs
     */
    template <uint32_t StackNum = 64, std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_GENERIC uint32_t query(const AABB& aabb,
                                CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        uint32_t stack[StackNum];
        return this->query(aabb, stack, StackNum, callback);
    }

    template <std::invocable<uint32_t> CallbackF = DefaultQueryCallback>
    MUDA_GENERIC uint32_t query(const AABB& aabb,
                                uint32_t*   stack,
                                uint32_t    stack_num,
                                CallbackF callback = DefaultQueryCallback{}) const noexcept
    {
        return this->query(
            aabb, [](const AABB& A, const AABB& B) { return A.intersects(B); }, stack, stack_num, callback);
    }


    /**
     * @brief check if the stack overflow occurs during the query.
     */
    bool stack_overflow() const noexcept { return m_stack_overflow; }

  private:
    uint32_t m_num_nodes;    // (# of internal node) + (# of leaves), 2N+1
    uint32_t m_num_objects;  // (# of leaves), the same as the number of objects

    auto_const_t<Node>* m_nodes;
    auto_const_t<AABB>* m_aabbs;

    MUDA_INLINE MUDA_GENERIC void check_index(const uint32_t idx) const noexcept
    {
        MUDA_KERNEL_ASSERT(idx < m_num_objects,
                           "BVHViewer[%s:%s]: index out of range, idx=%u, num_objects=%u",
                           this->name(),
                           this->kernel_name(),
                           idx,
                           m_num_objects);
    }

    MUDA_INLINE MUDA_GENERIC void stack_overflow_warning(uint32_t num_found,
                                                         uint32_t stack_num) const noexcept
    {
        if constexpr(muda::RUNTIME_CHECK_ON)
        {
            MUDA_KERNEL_WARN_WITH_LOCATION("BVHViewer[%s:%s]: stack overflow, num_found=%u, stack_num=%u, the return value may be invalid, enlarge the stack please.",
                                           this->name(),
                                           this->kernel_name(),
                                           num_found,
                                           stack_num);
        }
    }

    mutable bool m_stack_overflow = false;

    template <typename QueryType, typename IntersectF, typename CallbackF>
    MUDA_GENERIC uint32_t query(const QueryType& Q,
                                IntersectF       Intersect,
                                uint32_t*        stack,
                                uint32_t         stack_num,
                                CallbackF        Callback) const noexcept
    {
        uint32_t* stack_ptr = stack;
        uint32_t* stack_end = stack + stack_num;
        *stack_ptr++        = 0;  // root node is always 0

        if(m_num_objects == 1)
        {
            if(Intersect(m_aabbs[0], Q))
            {
                Callback(0);
                return 1;
            }
        }

        uint32_t num_found = 0;
        do
        {
            const uint32_t node  = *--stack_ptr;
            const uint32_t L_idx = m_nodes[node].left_idx;
            const uint32_t R_idx = m_nodes[node].right_idx;

            if(Intersect(m_aabbs[L_idx], Q))
            {
                const auto obj_idx = m_nodes[L_idx].object_idx;
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
            if(Intersect(m_aabbs[R_idx], Q))
            {
                const auto obj_idx = m_nodes[R_idx].object_idx;
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
                stack_overflow_warning(num_found, stack_num);
                break;
            }
        } while(stack < stack_ptr);
        return num_found;
    }
};

using LBVHViewer  = LBVHViewerT<false>;
using CLBVHViewer = LBVHViewerT<true>;

class LBVH
{
    using Node = detail::LBVHNode;

  public:
    using AABB        = Eigen::AlignedBox<Float, 3>;
    using MortonIndex = detail::LBVHMortonIndex;

    // now we only use default stream
    void build(muda::CBufferView<AABB> aabbs, muda::Stream& s = muda::Stream::Default())
    {
        using namespace muda;

        if(aabbs.size() == 0)
            return;

        const uint32_t num_objects        = aabbs.size();
        const uint32_t num_internal_nodes = num_objects - 1;
        const uint32_t leaf_start         = num_internal_nodes;
        const uint32_t num_nodes          = num_objects * 2 - 1;

        AABB default_aabb;
        resize(s, m_aabbs, num_nodes);
        BufferLaunch(s).fill(m_aabbs.view(), default_aabb);

        resize(s, m_sorted_mortons, num_objects);
        resize(s, m_sorted_mortons, num_objects);

        resize(s, m_indices, num_objects);
        resize(s, m_new_to_old, num_objects);

        resize(s, m_mortons, num_objects);
        resize(s, m_morton_idx, num_objects);

        Node default_node;
        m_nodes.resize(num_nodes, default_node);
        resize(s, m_nodes, num_nodes);

        resize(s, m_flags, num_objects);
        BufferLaunch(s).fill(m_flags.view(), 0);

        // 1) setup aabbs
        //BufferLaunch(s).copy(aabbs, aabbs);

        // 2) get max aabb
        DeviceReduce(s).Reduce(
            aabbs.data(),
            m_max_aabb.data(),
            aabbs.size(),
            [] CUB_RUNTIME_FUNCTION(const AABB& a, const AABB& b)
            { return a.merged(b); },
            default_aabb);

        // 3) calculate morton code
        on(s)
            .next<ParallelFor>()
            .kernel_name("LBVH::MortonCode")
            .apply(num_objects,
                   [max_aabb = m_max_aabb.viewer().name("max_aabb"),
                    aabbs    = aabbs.viewer().name("filled_aabbs"),
                    mortons = m_mortons.viewer().name("mortons")] __device__(int i) mutable
                   {
                       Vector3 p = aabbs(i).center();
                       p -= max_aabb->min();
                       p.array() /= max_aabb->sizes().array();
                       mortons(i) = detail::morton_code(p);
                   });

        // 4) sort morton code
        on(s)
            .next<ParallelFor>()
            .kernel_name("LBVH::Iota")
            .apply(m_indices.size(),
                   [indices = m_indices.viewer()] __device__(int i) mutable
                   { indices(i) = i; });

        // 5) sort morton code
        DeviceRadixSort(s).SortPairs(m_mortons.data(),
                                     m_sorted_mortons.data(),
                                     m_indices.data(),
                                     m_new_to_old.data(),
                                     num_objects);

        // 6) expand morton code to 64bit, the last 32bit is the index
        on(s)
            .next<ParallelFor>()
            .kernel_name("LBVH::ExpandMorton")
            .apply(m_mortons.size(),
                   [morton64s = m_morton_idx.viewer().name("morton64s"),
                    mortons   = m_sorted_mortons.viewer().name("mortons"),
                    indices = m_new_to_old.viewer().name("indices")] __device__(int i) mutable
                   {
                       MortonIndex morton{mortons(i), indices(i)};
                       morton64s(i) = morton;
                   });

        // 7) setup leaf nodes
        auto leaf_aabbs = m_aabbs.view(leaf_start);  // offset = leaf_start
        auto leaf_nodes = m_nodes.view(leaf_start);  // offset = leaf_start
        on(s)
            .next<ParallelFor>()
            .kernel_name("LBVH::SetupLeafNodes")
            .apply(num_objects,
                   [leaf_nodes = leaf_nodes.viewer().name("leaf_nodes"),
                    indices    = m_new_to_old.viewer().name("indices"),
                    aabbs      = aabbs.viewer().name("aabbs"),
                    sorted_aabbs = leaf_aabbs.viewer().name("sorted_aabbs")] __device__(int i) mutable
                   {
                       Node node;
                       node.parent_idx = 0xFFFFFFFF;
                       node.left_idx   = 0xFFFFFFFF;
                       node.right_idx  = 0xFFFFFFFF;
                       node.object_idx = indices(i);
                       leaf_nodes(i)   = node;
                       sorted_aabbs(i) = aabbs(node.object_idx);
                   });

        // 8) construct internal nodes
        on(s)
            .next<ParallelFor>()
            .kernel_name("LBVH::ConstructInternalNodes")
            .apply(num_internal_nodes,
                   [nodes     = m_nodes.viewer().name("nodes"),
                    morton64s = m_morton_idx.viewer().name("morton64s"),
                    num_objects] __device__(int idx) mutable
                   {
                       nodes(idx).object_idx = 0xFFFFFFFF;  //  internal nodes

                       const uint2 ij =
                           detail::determine_range(morton64s, num_objects, idx);
                       const int gamma =
                           detail::find_split(morton64s, num_objects, ij.x, ij.y);

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

        // 9) calculate the AABB of internal nodes
        auto internal_aabbs = m_aabbs.view(0, num_internal_nodes);
        on(s)
            .next<ParallelFor>()
            .kernel_name("LBVH::CalculateInternalAABB")
            .apply(num_objects,
                   [nodes = m_nodes.cviewer().name("nodes"),
                    aabbs = m_aabbs.viewer().name("aabbs"),
                    flags = m_flags.viewer().name("flags"),
                    leaf_start] __device__(int I) mutable
                   {
                       auto leaf_idx = I + leaf_start;
                       auto parent   = nodes(leaf_idx).parent_idx;

                       while(parent != 0xFFFFFFFF)  // means idx == 0
                       {
                           const int old = muda::atomic_cas(&flags(parent), 0, 1);
                           if(old == 0)
                           {
                               // this is the first thread entered here.
                               // wait the other thread from the other child node.
                               return;
                           }
                           MUDA_KERNEL_ASSERT(old == 1, "old=%d", old);
                           // here, the flag has already been 1. it means that this
                           // thread is the 2nd thread. merge AABB of both childlen.

                           const auto lidx = nodes(parent).left_idx;
                           const auto ridx = nodes(parent).right_idx;
                           const auto lbox = aabbs(lidx);
                           const auto rbox = aabbs(ridx);
                           aabbs(parent)   = lbox.merged(rbox);

                           // look the next parent...
                           parent = nodes(parent).parent_idx;
                       }
                   });
    }

    auto viewer() noexcept
    {
        return LBVHViewer{(uint32_t)m_nodes.size(),
                          (uint32_t)m_mortons.size(),
                          m_nodes.data(),
                          m_aabbs.data()};
    }

    auto viewer() const noexcept
    {
        return CLBVHViewer{(uint32_t)m_nodes.size(),
                           (uint32_t)m_mortons.size(),
                           m_nodes.data(),
                           m_aabbs.data()};
    }
    template <typename T>
    void resize(muda::Stream& s, muda::DeviceBuffer<T>& V, size_t size)
    {
        if(size > V.capacity())
            muda::BufferLaunch(s).reserve(V, size * m_resize_factor);
        muda::BufferLaunch(s).resize(V, size);
    }

    muda::DeviceBuffer<AABB>        m_aabbs;
    muda::DeviceBuffer<uint32_t>    m_mortons;
    muda::DeviceBuffer<uint32_t>    m_sorted_mortons;
    muda::DeviceBuffer<uint32_t>    m_indices;
    muda::DeviceBuffer<uint32_t>    m_new_to_old;
    muda::DeviceBuffer<MortonIndex> m_morton_idx;
    muda::DeviceBuffer<int>         m_flags;
    muda::DeviceBuffer<Node>        m_nodes;
    muda::DeviceVar<AABB>           m_max_aabb;

    Float m_resize_factor = 1.5;
};
}  // namespace uipc::test::backend::cuda


using namespace uipc;
using namespace uipc::test::backend::cuda;
using namespace uipc::geometry;
using namespace muda;


std::vector<Vector2i> lbvh_cp(span<const LBVH::AABB> aabbs)
{
    DeviceBuffer<LBVH::AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    LBVH lbvh;
    lbvh.build(d_aabbs);

    DeviceBuffer<IndexT> counts(aabbs.size() + 1ull);
    DeviceBuffer<IndexT> offsets(aabbs.size() + 1ull);

    for(int i = 0; i < aabbs.size(); ++i)
    {
        auto aabb = aabbs[i];
        std::cout << "[" << aabb.min().transpose() << "],"
                  << "[" << aabb.max().transpose() << "]" << std::endl;
    }

    ParallelFor()
        .kernel_name("LBVHTest::Query")
        .apply(aabbs.size(),
               [lbvh  = lbvh.viewer().name("lbvh"),
                aabbs = d_aabbs.viewer().name("aabbs"),
                counts = counts.viewer().name("counts")] __device__(int i) mutable
               {
                   auto N = aabbs.total_size();

                   auto aabb  = aabbs(i);
                   auto count = 0;
                   lbvh.query(aabb,
                              [&](uint32_t id)
                              {
                                  if(id > i)
                                      count++;
                              });
                   counts(i) = count;

                   if(i == 0)
                   {
                       counts(N) = 0;
                   }
               });

    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());
    IndexT total;
    offsets.view(aabbs.size()).copy_to(&total);

    DeviceBuffer<Vector2i> pairs(total);


    ParallelFor()
        .kernel_name("LBVHTest::Query")
        .apply(aabbs.size(),
               [lbvh    = lbvh.viewer().name("lbvh"),
                aabbs   = d_aabbs.viewer().name("aabbs"),
                counts  = counts.viewer().name("counts"),
                offsets = offsets.viewer().name("offsets"),
                pairs = pairs.viewer().name("pairs")] __device__(int i) mutable
               {
                   auto N = aabbs.total_size();

                   auto aabb   = aabbs(i);
                   auto count  = counts(i);
                   auto offset = offsets(i);

                   auto pair = pairs.subview(offset, count);
                   int  j    = 0;
                   lbvh.query(aabb,
                              [&](uint32_t id)
                              {
                                  if(id > i)
                                      pair(j++) = Vector2i(i, id);
                              });
               });

    std::vector<Vector2i> pairs_host;
    pairs.copy_to(pairs_host);

    //std::vector<LBVH::AABB> aabbs_host;
    //lbvh.m_aabbs.copy_to(aabbs_host);
    //for(auto&& [i, aabb] : enumerate(aabbs_host))
    //{
    //    std::cout << "[" << aabb.min().transpose() << "],"
    //              << "[" << aabb.max().transpose() << "]" << std::endl;
    //}

    //std::vector<detail::LBVHNode> nodes_host;
    //lbvh.m_nodes.copy_to(nodes_host);
    //for(auto&& [i, node] : enumerate(nodes_host))
    //{
    //    std::cout << "node=" << i << "[" << aabbs_host[i].min().transpose() << "],"
    //              << "[" << aabbs_host[i].max().transpose() << "]"
    //              << ", parent=" << node.parent_idx
    //              << ", left=" << node.left_idx << ", right=" << node.right_idx
    //              << ", obj=" << node.object_idx << std::endl;
    //}

    return pairs_host;
}

std::vector<Vector2i> brute_froce_cp(span<const LBVH::AABB> aabbs)
{
    std::vector<Vector2i> pairs;
    for(auto&& [i, aabb0] : enumerate(aabbs))
    {
        for(int j = i + 1; j < aabbs.size(); ++j)
        {
            auto aabb1 = aabbs[j];
            if(aabb1.intersects(aabb0))
            {
                pairs.push_back(Vector2i(i, j));
            }
        }
    }
    return pairs;
}


void brute_froce_gpu(span<const LBVH::AABB> aabbs)
{
    DeviceBuffer<LBVH::AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    ParallelFor()
        .kernel_name("BruteForce::Query")
        .apply(aabbs.size(),
               [aabbs = d_aabbs.viewer().name("aabbs")] __device__(int i) mutable
               {
                   auto N = aabbs.total_size();

                   auto aabb0 = aabbs(i);
                   for(int j = i + 1; j < N; ++j)
                   {
                       auto aabb1 = aabbs(j);
                       if(aabb1.intersects(aabb0))
                       {
                           printf("i=%d, j=%d\n", i, j);
                       }
                   }
               });
}

SimplicialComplex tet()
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    return tetmesh(Vs, Ts);
}


void lbvh_test(const SimplicialComplex& mesh)
{
    auto pos_view = mesh.positions().view();
    auto tri_view = mesh.triangles().topo().view();

    //tri_view = tri_view.subspan(0, 5);

    std::vector<LBVH::AABB> aabbs(tri_view.size());
    for(auto&& [i, tri] : enumerate(tri_view))
    {
        auto p0 = pos_view[tri[0]];
        auto p1 = pos_view[tri[1]];
        auto p2 = pos_view[tri[2]];
        aabbs[i].extend(p0).extend(p1).extend(p2);
        aabbs[i].max() += Vector3{0.1, 0.1, 0.1};
        aabbs[i].min() -= Vector3{0.1, 0.1, 0.1};
    }


    auto lbvh_pairs = lbvh_cp(aabbs);


    auto bf_pairs = brute_froce_cp(aabbs);

    // brute_froce_gpu(aabbs);


    auto compare = [](const Vector2i& lhs, const Vector2i& rhs)
    { return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]); };

    std::ranges::sort(lbvh_pairs, compare);
    std::ranges::sort(bf_pairs, compare);


    auto check_unique = [](auto begin, auto end)
    {
        for(auto it = begin; it != end; ++it)
        {
            if(it + 1 != end && *it == *(it + 1))
            {
                return false;
            }
        }
        return true;
    };

    CHECK(check_unique(lbvh_pairs.begin(), lbvh_pairs.end()));

    bool same = lbvh_pairs == bf_pairs;
    CHECK(same);

    if(!same)
    {
        std::list<Vector2i> diff;

        std::set_difference(lbvh_pairs.begin(),
                            lbvh_pairs.end(),
                            bf_pairs.begin(),
                            bf_pairs.end(),
                            std::back_inserter(diff),
                            compare);

        std::cout << "lbvh_pairs.size()=" << lbvh_pairs.size() << std::endl;
        std::cout << "bf_pairs.size()=" << bf_pairs.size() << std::endl;
        std::cout << "diff:" << std::endl;
        for(auto&& [i, p] : enumerate(diff))
        {
            std::cout << "i=" << i << ", p=[" << p[0] << ", " << p[1] << "]" << std::endl;
        }
    }
}


TEST_CASE("lbvh", "[muda]")
{

    SECTION("tet")
    {
        lbvh_test(tet());
    }

    SECTION("cube.obj")
    {
        SimplicialComplexIO io;
        auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        lbvh_test(mesh);
    }

    SECTION("cube.msh")
    {
        SimplicialComplexIO io;
        auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }
}
