#pragma once
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <cuda_tool/cuda_tool.h>
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
// InfoStacklessBVH: LBVH with per-node body/contact ID metadata for early
// subtree culling. The traversal kernels pre-load per-query bid/cid into
// shared memory once per thread before the hot loop, eliminating repeated
// global memory reads of query_bid/query_cid on every visited node.
class InfoStacklessBVH
{
  public:
    // NodePredInfo carries pre-loaded query bid/cid so the user's NodePred
    // does NOT need to capture and read global arrays per node.
    class NodePredInfo
    {
      public:
        IndexT query_id  = -1;
        IndexT query_bid = -1;  // pre-loaded from shared memory
        IndexT query_cid = -1;  // pre-loaded from shared memory
        IndexT node_bid  = -1;
        IndexT node_cid  = -1;

        NodePredInfo() = default;
        UIPC_GENERIC NodePredInfo(IndexT query_id, IndexT query_bid, IndexT query_cid, IndexT node_bid, IndexT node_cid)
            : query_id(query_id)
            , query_bid(query_bid)
            , query_cid(query_cid)
            , node_bid(node_bid)
            , node_cid(node_cid)
        {
        }
    };

    // LeafPredInfo carries the primitive pair plus their pre-resolved bid/cid.
    // bid/cid come from shared memory (query side) and the BVH node struct
    // (leaf side), so the user's LeafPred avoids global v2b / contact_ids reads
    // for the body-pair check.
    // NOTE: the bid/cid check is NOT redundant with NodePred — when an internal
    // node has bid/cid == -1 (spans multiple bodies/contacts) NodePred cannot
    // cull it; the exact check must happen here at the leaf.
    class LeafPredInfo
    {
      public:
        IndexT i     = -1;
        IndexT j     = -1;
        IndexT bid_i = -1;  // body ID of primitive i
        IndexT cid_i = -1;  // contact ID of primitive i
        IndexT bid_j = -1;  // body ID of primitive j
        IndexT cid_j = -1;  // contact ID of primitive j

        LeafPredInfo() = default;
        UIPC_GENERIC LeafPredInfo(IndexT i, IndexT j, IndexT bid_i, IndexT cid_i, IndexT bid_j, IndexT cid_j)
            : i(i)
            , j(j)
            , bid_i(bid_i)
            , cid_i(cid_i)
            , bid_j(bid_j)
            , cid_j(cid_j)
        {
        }
    };

    class QueryBuffer
    {
      public:
        QueryBuffer() { m_pairs.resize(50 * 1024); }

        auto  view() const noexcept { return m_pairs.view(0, m_size); }
        void  reserve(size_t size) { m_pairs.resize(size); }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }

      public:
        friend class InfoStacklessBVH;
        SizeT                                 m_size = 0;
        cuda_tool::DeviceBuffer<Vector2i>     m_pairs;
        cuda_tool::DeviceBuffer<unsigned int> m_queryMtCode;
        cuda_tool::DeviceVar<AABB>            m_querySceneBox;
        cuda_tool::DeviceBuffer<int>          m_querySortedId;
        cuda_tool::DeviceVar<int>             m_cpNum;

        void build(cuda_tool::CBufferView<AABB> aabbs);
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

    InfoStacklessBVH(cuda_tool::Stream& stream = cuda_tool::Stream::Default()) noexcept;

    void build(cuda_tool::CBufferView<AABB>   aabbs,
               cuda_tool::CBufferView<IndexT> BIDs,
               cuda_tool::CBufferView<IndexT> CIDs);
    void build(cuda_tool::CBufferView<AABB> aabbs);

    template <typename NodePred, typename LeafPred>
    void detect(cuda_tool::CBuffer2DView<IndexT> cmts, NodePred np, LeafPred lp, QueryBuffer& qbuffer);

    template <typename NodePred, typename LeafPred>
    void query(cuda_tool::CBufferView<AABB>     query_aabbs,
               cuda_tool::CBufferView<IndexT>   query_BIDs,
               cuda_tool::CBufferView<IndexT>   query_CIDs,
               cuda_tool::CBuffer2DView<IndexT> cmts,
               NodePred                         np,
               LeafPred                         lp,
               QueryBuffer&                     qbuffer);

    Config&       config() noexcept { return m_impl.config; }
    const Config& config() const noexcept { return m_impl.config; }

  public:
    class Impl
    {
      public:
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
        void        propagateInformativeMetadata(int intSize);
        void        build(cuda_tool::CBufferView<AABB>   aabbs,
                          cuda_tool::CBufferView<IndexT> bids,
                          cuda_tool::CBufferView<IndexT> cids);

        // Pre-loads query bid/cid into shared memory before the traversal loop.
        // node_cull receives NodePredInfo with query_bid/query_cid from SMem.
        template <typename NodeCull, typename PairPred>
        void stacklessSelf(NodeCull                        node_cull,
                           PairPred                        pair_pred,
                           cuda_tool::VarView<int>         cpNum,
                           cuda_tool::BufferView<Vector2i> buffer);

        // Pre-loads query_bids/query_cids into shared memory before the traversal loop.
        // node_cull receives NodePredInfo with query_bid/query_cid from SMem.
        template <typename NodeCull, typename PairPred>
        void stacklessOther(NodeCull                        node_cull,
                            PairPred                        pair_pred,
                            cuda_tool::CBufferView<AABB>    query_aabbs,
                            cuda_tool::CBufferView<IndexT>  query_bids,
                            cuda_tool::CBufferView<IndexT>  query_cids,
                            cuda_tool::CBufferView<int>     query_sorted_id,
                            cuda_tool::VarView<int>         cpNum,
                            cuda_tool::BufferView<Vector2i> buffer);

        cuda_tool::CBufferView<AABB>      objs;
        cuda_tool::CBufferView<IndexT>    bids;
        cuda_tool::CBufferView<IndexT>    cids;
        cuda_tool::DeviceVar<AABB>        scene_box;
        cuda_tool::DeviceVector<uint32_t> flags;
        cuda_tool::DeviceVector<uint32_t> mtcode;
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
        cuda_tool::DeviceVector<IndexT>   ext_bid;
        cuda_tool::DeviceVector<IndexT>   ext_cid;
        cuda_tool::DeviceVector<IndexT>   int_bid;
        cuda_tool::DeviceVector<IndexT>   int_cid;
        cuda_tool::DeviceVector<Node>     nodes;
        Config                            config;
    };

  private:
    cuda_tool::CBufferView<AABB>   m_aabbs;
    cuda_tool::CBufferView<IndexT> m_BIDs;
    cuda_tool::CBufferView<IndexT> m_CIDs;
    Impl                           m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/info_stackless_bvh.inl"
