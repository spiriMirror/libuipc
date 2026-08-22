/**
 * @file stackless_bvh.h
 * 
 * @brief UIPC Compatible Version of Stackless BVH for AABB overlap detection (safe muda-style)
 * 
 * References:
 * 
 * Thanks to the original authors of the following repositories for their excellent implementations of Stackless BVH!
 * 
 * - https://github.com/ZiXuanVickyLu/culbvh
 * - https://github.com/jerry060599/KittenGpuLBVH
 * 
 */

#pragma once
#include <uipc/common/logger.h>
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <cuda_tool/cuda_tool.h>

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
        Float reserve_ratio;
        Config()
            : reserve_ratio(1.2)
        {
        }
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
        SizeT                             m_size = 0;
        cuda_tool::DeviceBuffer<Vector2i> m_pairs;

        cuda_tool::DeviceBuffer<unsigned int> m_queryMtCode;
        cuda_tool::DeviceVar<AABB>            m_querySceneBox;
        cuda_tool::DeviceBuffer<int>          m_querySortedId;
        cuda_tool::DeviceVar<int>             m_cpNum;

        void  build(cuda_tool::CBufferView<AABB> aabbs);
        SizeT query_count() { return m_queryMtCode.size(); }
    };

    struct /*__align__(16) */ Node
    {
        IndexT lc;
        IndexT escape;
        AABB   bound;
    };

    StacklessBVH(Config config = Config{}) { m_impl.config = config; }

    ~StacklessBVH() = default;

    struct DefaultQueryCallback
    {
        UIPC_GENERIC bool operator()(IndexT i, IndexT j) const { return true; }
    };

    /**
     * @brief Build the Stackless BVH from given AABBs
     * 
     * @param aabbs Input AABBs, aabbs must be kept valid during the lifetime of this BVH
     */
    void build(cuda_tool::CBufferView<AABB> aabbs);

    /**
     * @brief Detect overlapping AABB pairs in the BVH
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
    void query(cuda_tool::CBufferView<AABB> aabbs, Pred callback, QueryBuffer& qbuffer);


  public:
    class Impl
    {
      public:
        void build(cuda_tool::CBufferView<AABB> aabbs);
        template <typename Pred>
        void StacklessCDSharedSelf(Pred                            pred,
                                   cuda_tool::VarView<int>         cpNum,
                                   cuda_tool::BufferView<Vector2i> buffer);
        template <typename Pred>
        void StacklessCDSharedOther(Pred                         pred,
                                    cuda_tool::CBufferView<AABB> query_aabbs,
                                    cuda_tool::CBufferView<int> query_sorted_id,
                                    cuda_tool::VarView<int>     cpNum,
                                    cuda_tool::BufferView<Vector2i> buffer);


        static void calcMaxBVFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                     cuda_tool::VarView<AABB>     scene_box);
        static void calcMCsFromBox(cuda_tool::CBufferView<AABB>    aabbs,
                                   cuda_tool::CVarView<AABB>       scene_box,
                                   cuda_tool::BufferView<uint32_t> codes);
        void        calcInverseMapping();
        void        buildPrimitivesFromBox(cuda_tool::CBufferView<AABB> aabbs);
        void        calcExtNodeSplitMetrics();
        void        buildIntNodes(int size);
        void        calcIntNodeOrders(int size);
        void        updateBvhExtNodeLinks(int size);
        void        reorderNode(int intSize);


        cuda_tool::CBufferView<AABB> objs;  // external AABBs, should be kept valid
        cuda_tool::DeviceVar<AABB>        scene_box;  // external bounding boxes
        cuda_tool::DeviceVector<uint32_t> flags;
        cuda_tool::DeviceVector<uint32_t> mtcode;  // external morton codes
        cuda_tool::DeviceVector<int32_t>  sorted_id;
        cuda_tool::DeviceVector<int32_t>  primMap;
        cuda_tool::DeviceVector<int>      metric;
        cuda_tool::DeviceVector<uint32_t> count;
        cuda_tool::DeviceVector<int>      tkMap;
        cuda_tool::DeviceVector<uint32_t> offsetTable;

        cuda_tool::DeviceVector<AABB>     ext_aabb;
        cuda_tool::DeviceVector<int>      ext_idx;
        cuda_tool::DeviceVector<int>      ext_lca;
        cuda_tool::DeviceVector<uint32_t> ext_mark;
        cuda_tool::DeviceVector<uint32_t> ext_par;

        cuda_tool::DeviceVector<int>      int_lc;
        cuda_tool::DeviceVector<int>      int_rc;
        cuda_tool::DeviceVector<int>      int_par;
        cuda_tool::DeviceVector<int>      int_range_x;
        cuda_tool::DeviceVector<int>      int_range_y;
        cuda_tool::DeviceVector<uint32_t> int_mark;
        cuda_tool::DeviceVector<AABB>     int_aabb;

        cuda_tool::DeviceVector<ulonglong2> quantNode;
        cuda_tool::DeviceVector<Node>       nodes;

        Config config;
    };

  private:
    Impl m_impl;
};

}  // namespace uipc::backend::cuda

#include "details/stackless_bvh.inl"