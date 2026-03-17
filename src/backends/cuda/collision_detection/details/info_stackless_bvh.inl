namespace uipc::backend::cuda
{
inline InfoStacklessBVH::InfoStacklessBVH(muda::Stream& stream) noexcept
{
    (void)stream;
}

inline void InfoStacklessBVH::build(muda::CBufferView<AABB>   aabbs,
                                    muda::CBufferView<IndexT> BIDs,
                                    muda::CBufferView<IndexT> CIDs)
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

    m_stackless_bvh.build(aabbs);
}

inline void InfoStacklessBVH::build(muda::CBufferView<AABB> aabbs)
{
    m_aabbs = aabbs;
    m_BIDs  = {};
    m_CIDs  = {};
    m_stackless_bvh.build(aabbs);
}

template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVH::detect(muda::CBuffer2DView<IndexT> cmts,
                                     NodePred                     np,
                                     LeafPred                     lp,
                                     QueryBuffer&                 qbuffer)
{
    if(m_aabbs.size() == 0)
    {
        m_stackless_bvh.detect([] __device__(IndexT, IndexT) { return false; }, qbuffer.m_qbuffer);
        return;
    }

    UIPC_ASSERT(m_aabbs.size() == m_BIDs.size(),
                "AABB and BID size mismatch. aabbs=%zu, bids=%zu",
                m_aabbs.size(),
                m_BIDs.size());
    UIPC_ASSERT(m_aabbs.size() == m_CIDs.size(),
                "AABB and CID size mismatch. aabbs=%zu, cids=%zu",
                m_aabbs.size(),
                m_CIDs.size());

    constexpr IndexT invalid_index = static_cast<IndexT>(-1);

    m_stackless_bvh.detect(
        [BIDs = m_BIDs.viewer().name("BIDs"),
         CIDs = m_CIDs.viewer().name("CIDs"),
         cmts = cmts.viewer().name("cmts"),
         np   = np,
         lp   = lp] __device__(IndexT i, IndexT j)
        {
            if(j <= i)
                return false;

            const auto query_bid = BIDs(i);
            const auto query_cid = CIDs(i);
            const auto node_bid  = BIDs(j);
            const auto node_cid  = CIDs(j);

            const bool cull_by_bid = node_bid != invalid_index && query_bid == node_bid;
            if(cull_by_bid)
                return false;

            const bool has_valid_cid = query_cid != invalid_index && node_cid != invalid_index;
            if(has_valid_cid && !cmts(query_cid, node_cid))
                return false;

            if(!np(NodePredInfo{i, node_bid, node_cid}))
                return false;

            return lp(i, j);
        },
        qbuffer.m_qbuffer);
}

template <std::invocable<IndexT, IndexT> Pred>
inline void InfoStacklessBVH::detect(Pred callback, QueryBuffer& qbuffer)
{
    m_stackless_bvh.detect(callback, qbuffer.m_qbuffer);
}

template <typename NodePred, typename LeafPred>
inline void InfoStacklessBVH::query(muda::CBufferView<AABB>   query_aabbs,
                                    muda::CBufferView<IndexT> query_BIDs,
                                    muda::CBufferView<IndexT> query_CIDs,
                                    muda::CBuffer2DView<IndexT> cmts,
                                    NodePred                    np,
                                    LeafPred                    lp,
                                    QueryBuffer&                qbuffer)
{
    if(m_aabbs.size() == 0 || query_aabbs.size() == 0)
    {
        m_stackless_bvh.query(query_aabbs,
                              [] __device__(IndexT, IndexT) { return false; },
                              qbuffer.m_qbuffer);
        return;
    }

    UIPC_ASSERT(query_aabbs.size() == query_BIDs.size(),
                "Query AABB and BID size mismatch. aabbs=%zu, bids=%zu",
                query_aabbs.size(),
                query_BIDs.size());
    UIPC_ASSERT(query_aabbs.size() == query_CIDs.size(),
                "Query AABB and CID size mismatch. aabbs=%zu, cids=%zu",
                query_aabbs.size(),
                query_CIDs.size());

    constexpr IndexT invalid_index = static_cast<IndexT>(-1);

    m_stackless_bvh.query(
        query_aabbs,
        [query_BIDs = query_BIDs.viewer().name("query_BIDs"),
         query_CIDs = query_CIDs.viewer().name("query_CIDs"),
         BIDs       = m_BIDs.viewer().name("BIDs"),
         CIDs       = m_CIDs.viewer().name("CIDs"),
         cmts       = cmts.viewer().name("cmts"),
         np         = np,
         lp         = lp] __device__(IndexT i, IndexT j)
        {
            const auto query_bid = query_BIDs(i);
            const auto query_cid = query_CIDs(i);
            const auto node_bid  = BIDs(j);
            const auto node_cid  = CIDs(j);

            const bool cull_by_bid = node_bid != invalid_index && query_bid == node_bid;
            if(cull_by_bid)
                return false;

            const bool has_valid_cid = query_cid != invalid_index && node_cid != invalid_index;
            if(has_valid_cid && !cmts(query_cid, node_cid))
                return false;

            if(!np(NodePredInfo{i, node_bid, node_cid}))
                return false;

            return lp(i, j);
        },
        qbuffer.m_qbuffer);
}

template <std::invocable<IndexT, IndexT> Pred>
inline void InfoStacklessBVH::query(muda::CBufferView<AABB> query_aabbs,
                                    Pred                    callback,
                                    QueryBuffer&            qbuffer)
{
    m_stackless_bvh.query(query_aabbs, callback, qbuffer.m_qbuffer);
}
}  // namespace uipc::backend::cuda
