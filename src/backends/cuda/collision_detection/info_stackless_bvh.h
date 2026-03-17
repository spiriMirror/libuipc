#pragma once
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <collision_detection/stackless_bvh.h>
#include <muda/buffer.h>
#include <uipc/common/log.h>
#include <concepts>

namespace uipc::backend::cuda
{
class InfoStacklessBVH
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
        auto  view() const noexcept { return m_qbuffer.view(); }
        void  reserve(size_t size) { m_qbuffer.reserve(size); }
        SizeT size() const noexcept { return m_qbuffer.size(); }
        auto  viewer() const noexcept { return m_qbuffer.viewer(); }

      private:
        friend class InfoStacklessBVH;
        StacklessBVH::QueryBuffer m_qbuffer;
    };

    InfoStacklessBVH(muda::Stream& stream = muda::Stream::Default()) noexcept;

    void build(muda::CBufferView<AABB>   aabbs,
               muda::CBufferView<IndexT> BIDs,
               muda::CBufferView<IndexT> CIDs);
    void build(muda::CBufferView<AABB> aabbs);

    template <typename NodePred, typename LeafPred>
    void detect(muda::CBuffer2DView<IndexT> cmts, NodePred np, LeafPred lp, QueryBuffer& qbuffer);
    template <std::invocable<IndexT, IndexT> Pred = DefaultQueryCallback>
    void detect(Pred callback, QueryBuffer& qbuffer);

    template <typename NodePred, typename LeafPred>
    void query(muda::CBufferView<AABB>   query_aabbs,
               muda::CBufferView<IndexT> query_BIDs,
               muda::CBufferView<IndexT> query_CIDs,
               muda::CBuffer2DView<IndexT> cmts,
               NodePred np,
               LeafPred lp,
               QueryBuffer& qbuffer);
    template <std::invocable<IndexT, IndexT> Pred = DefaultQueryCallback>
    void query(muda::CBufferView<AABB> query_aabbs, Pred callback, QueryBuffer& qbuffer);

  private:
    muda::CBufferView<AABB>   m_aabbs;
    muda::CBufferView<IndexT> m_BIDs;
    muda::CBufferView<IndexT> m_CIDs;
    StacklessBVH              m_stackless_bvh;
};
}  // namespace uipc::backend::cuda

#include "details/info_stackless_bvh.inl"
