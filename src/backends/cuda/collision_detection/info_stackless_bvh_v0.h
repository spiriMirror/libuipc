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
// InfoStacklessBVHV0: LBVH with per-node body/contact ID metadata.
// V0 (baseline): node_cull receives only (query_id, node_bid, node_cid);
// the user's NodePred must read query_bid/query_cid from global memory via query_id.
// Compare with InfoStacklessBVH which pre-loads query bid/cid into shared memory.
class InfoStacklessBVHV0
{
  public:
    class NodePredInfo
    {
      public:
        IndexT query_id = -1;
        IndexT node_bid = -1;
        IndexT node_cid = -1;

        NodePredInfo() = default;
        UIPC_GENERIC NodePredInfo(IndexT query_id, IndexT node_bid, IndexT node_cid)
            : query_id(query_id)
            , node_bid(node_bid)
            , node_cid(node_cid)
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
        friend class InfoStacklessBVHV0;
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

    InfoStacklessBVHV0(cuda_tool::Stream& stream = cuda_tool::Stream::Default()) noexcept;

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

        template <typename NodeCull, typename PairPred>
        void stacklessSelf(NodeCull                        node_cull,
                           PairPred                        pair_pred,
                           cuda_tool::VarView<int>         cpNum,
                           cuda_tool::BufferView<Vector2i> buffer);

        template <typename NodeCull, typename PairPred>
        void stacklessOther(NodeCull                        node_cull,
                            PairPred                        pair_pred,
                            cuda_tool::CBufferView<AABB>    query_aabbs,
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

#include "details/info_stackless_bvh_v0.inl"
