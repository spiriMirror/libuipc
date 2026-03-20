#pragma once
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <muda/buffer.h>
#include <uipc/common/log.h>
#include <concepts>
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
// InfoStacklessBVHV0: LBVH with per-node body/contact ID metadata.
// V0 (baseline): node_cull receives only (query_id, node_bid, node_cid);
// the user's NodePred must read query_bid/query_cid from global memory via query_id.
// Compare with InfoStacklessBVH which pre-loads query bid/cid into shared memory.
class InfoStacklessBVHV0
{
  public:
    struct DefaultQueryCallback
    {
        MUDA_GENERIC bool operator()(IndexT i, IndexT j) const
        {
            (void)i;
            (void)j;
            return true;
        }
    };

    class NodePredInfo
    {
      public:
        IndexT query_id = -1;
        IndexT node_bid = -1;
        IndexT node_cid = -1;

        NodePredInfo() = default;
        MUDA_GENERIC NodePredInfo(IndexT query_id, IndexT node_bid, IndexT node_cid)
            : query_id(query_id)
            , node_bid(node_bid)
            , node_cid(node_cid)
        {
        }
    };

    class QueryBuffer
    {
      public:
        QueryBuffer()
        {
            m_pairs.resize(50 * 1024);
        }

        auto  view() const noexcept { return m_pairs.view(0, m_size); }
        void  reserve(size_t size) { m_pairs.resize(size); }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }

      public:
        friend class InfoStacklessBVHV0;
        SizeT                            m_size = 0;
        muda::DeviceBuffer<Vector2i>     m_pairs;
        muda::DeviceBuffer<unsigned int> m_queryMtCode;
        muda::DeviceVar<AABB>            m_querySceneBox;
        muda::DeviceBuffer<int>          m_querySortedId;
        muda::DeviceVar<int>             m_cpNum;

        void build(muda::CBufferView<AABB> aabbs);
    };

    struct Node
    {
        IndexT lc     = -1;
        IndexT escape = -1;
        AABB   bound;
        IndexT bid = -1;
        IndexT cid = -1;
    };

    class Config
    {
      public:
        Float reserve_ratio = 1.2;
    };

    InfoStacklessBVHV0(muda::Stream& stream = muda::Stream::Default()) noexcept;

    void build(muda::CBufferView<AABB>   aabbs,
               muda::CBufferView<IndexT> BIDs,
               muda::CBufferView<IndexT> CIDs);
    void build(muda::CBufferView<AABB> aabbs);

    template <typename NodePred, typename LeafPred>
    void detect(muda::CBuffer2DView<IndexT> cmts, NodePred np, LeafPred lp, QueryBuffer& qbuffer);
    template <std::invocable<IndexT, IndexT> Pred = DefaultQueryCallback>
    void detect(Pred callback, QueryBuffer& qbuffer);

    template <typename NodePred, typename LeafPred>
    void query(muda::CBufferView<AABB>      query_aabbs,
               muda::CBufferView<IndexT>    query_BIDs,
               muda::CBufferView<IndexT>    query_CIDs,
               muda::CBuffer2DView<IndexT>  cmts,
               NodePred                     np,
               LeafPred                     lp,
               QueryBuffer&                 qbuffer);
    template <std::invocable<IndexT, IndexT> Pred = DefaultQueryCallback>
    void query(muda::CBufferView<AABB> query_aabbs, Pred callback, QueryBuffer& qbuffer);

    Config&       config() noexcept { return m_impl.config; }
    const Config& config() const noexcept { return m_impl.config; }

  public:
    class Impl
    {
      public:
        static void calcMaxBVFromBox(muda::CBufferView<AABB> aabbs,
                                     muda::VarView<AABB>     scene_box);
        static void calcMCsFromBox(muda::CBufferView<AABB>    aabbs,
                                   muda::CVarView<AABB>       scene_box,
                                   muda::BufferView<uint32_t> codes);
        void        calcInverseMapping();
        void        buildPrimitivesFromBox(muda::CBufferView<AABB> aabbs);
        void        calcExtNodeSplitMetrics();
        void        buildIntNodes(int size);
        void        calcIntNodeOrders(int size);
        void        updateBvhExtNodeLinks(int size);
        void        reorderNode(int intSize);
        void        propagateInformativeMetadata(int intSize);
        void        build(muda::CBufferView<AABB>   aabbs,
                          muda::CBufferView<IndexT> bids,
                          muda::CBufferView<IndexT> cids);

        template <typename NodeCull, typename PairPred>
        void stacklessSelf(NodeCull                   node_cull,
                           PairPred                   pair_pred,
                           muda::VarView<int>         cpNum,
                           muda::BufferView<Vector2i> buffer);

        template <typename NodeCull, typename PairPred>
        void stacklessOther(NodeCull                    node_cull,
                            PairPred                    pair_pred,
                            muda::CBufferView<AABB>     query_aabbs,
                            muda::CBufferView<int>      query_sorted_id,
                            muda::VarView<int>          cpNum,
                            muda::BufferView<Vector2i>  buffer);

        muda::CBufferView<AABB>   objs;
        muda::CBufferView<IndexT> bids;
        muda::CBufferView<IndexT> cids;
        muda::DeviceVar<AABB>     scene_box;
        muda::DeviceVector<uint32_t> flags;
        muda::DeviceVector<uint32_t> mtcode;
        muda::DeviceVector<int32_t>  sorted_id;
        muda::DeviceVector<int32_t>  primMap;
        muda::DeviceVector<int>      metric;
        muda::DeviceVector<uint32_t> count;
        muda::DeviceVector<int>      tkMap;
        muda::DeviceVector<uint32_t> offsetTable;
        muda::DeviceVector<AABB>     ext_aabb;
        muda::DeviceVector<int>      ext_idx;
        muda::DeviceVector<int>      ext_lca;
        muda::DeviceVector<uint32_t> ext_mark;
        muda::DeviceVector<uint32_t> ext_par;
        muda::DeviceVector<int>      int_lc;
        muda::DeviceVector<int>      int_rc;
        muda::DeviceVector<int>      int_par;
        muda::DeviceVector<int>      int_range_x;
        muda::DeviceVector<int>      int_range_y;
        muda::DeviceVector<uint32_t> int_mark;
        muda::DeviceVector<AABB>     int_aabb;
        muda::DeviceVector<IndexT>   ext_bid;
        muda::DeviceVector<IndexT>   ext_cid;
        muda::DeviceVector<IndexT>   int_bid;
        muda::DeviceVector<IndexT>   int_cid;
        muda::DeviceVector<Node>     nodes;
        Config                       config;
    };

  private:
    muda::CBufferView<AABB>   m_aabbs;
    muda::CBufferView<IndexT> m_BIDs;
    muda::CBufferView<IndexT> m_CIDs;
    Impl                      m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/info_stackless_bvh_v0.inl"
