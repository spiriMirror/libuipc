/*
 * NVIDIA contributions: BVH refit/query-reuse interfaces and state; launch
 * streams of build, refit and query, and the warp-cooperative Other-query
 * configuration.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <type_define.h>
#include <collision_detection/aabb.h>
#include <cuda_tool/cuda_tool.h>
#include <uipc/common/log.h>
#include <concepts>

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
        QueryBuffer()
        {
            m_pairs.reserve_discard(50 * 1024);
            m_pairs.resize_discard(50 * 1024);
        }

        auto view() const noexcept { return m_pairs.view(0, m_size); }
        void reserve(size_t size)
        {
            m_pairs.reserve_discard(size);
            m_pairs.resize_discard(size);
        }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }

      public:
        friend class InfoStacklessBVH;
        SizeT                                 m_size = 0;
        cuda_tool::DeviceBuffer<Vector2i>     m_pairs;
        cuda_tool::DeviceBuffer<unsigned int> m_queryMtCode;
        cuda_tool::DeviceBuffer<unsigned int> m_querySortedMtCode;
        cuda_tool::DeviceVar<AABB>            m_querySceneBox;
        cuda_tool::DeviceBuffer<int>          m_queryId;
        cuda_tool::DeviceBuffer<int>          m_querySortedId;
        cuda_tool::DeviceVar<int>             m_cpNum;

        void build(cuda_tool::CBufferView<AABB> aabbs,
                   bool                         reuse_order = false,
                   cudaStream_t stream = cuda_tool::default_stream());
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
        // Other queries share each warp's queries among its lanes and batch
        // their leaf predicates. false selects the per-lane traversal, which
        // yields the same pair multiset in another order (kept as the
        // validation reference).
        bool warp_cooperative_query = true;
        // Queries per warp of the cooperative traversal (1..32); 0 chooses
        // the largest power of two that still leaves 32 warps per SM.
        int queries_per_warp = 0;
    };

    InfoStacklessBVH(cuda_tool::Stream& stream = cuda_tool::Stream::Default()) noexcept;

    // build/refit/launch_query enqueue all their work on `stream`.
    void build(cuda_tool::CBufferView<AABB>   aabbs,
               cuda_tool::CBufferView<IndexT> BIDs,
               cuda_tool::CBufferView<IndexT> CIDs,
               cudaStream_t stream = cuda_tool::default_stream());
    void build(cuda_tool::CBufferView<AABB> aabbs);

    // Reuse the existing Morton topology while updating bounds and informative
    // metadata. A false result leaves the tree untouched so callers can fall
    // back to a full build when the primitive count or cache is incompatible.
    bool refit(cuda_tool::CBufferView<AABB>   aabbs,
               cuda_tool::CBufferView<IndexT> BIDs,
               cuda_tool::CBufferView<IndexT> CIDs,
               cudaStream_t stream = cuda_tool::default_stream());

    template <typename NodePred, typename LeafPred>
    void detect(cuda_tool::CBuffer2DView<IndexT> cmts, NodePred np, LeafPred lp, QueryBuffer& qbuffer);

    template <typename NodePred, typename LeafPred>
    void launch_detect(cuda_tool::CBuffer2DView<IndexT> cmts,
                       NodePred                         np,
                       LeafPred                         lp,
                       QueryBuffer&                     qbuffer);

    template <typename NodePred, typename LeafPred>
    void query(cuda_tool::CBufferView<AABB>     query_aabbs,
               cuda_tool::CBufferView<IndexT>   query_BIDs,
               cuda_tool::CBufferView<IndexT>   query_CIDs,
               cuda_tool::CBuffer2DView<IndexT> cmts,
               NodePred                         np,
               LeafPred                         lp,
               QueryBuffer&                     qbuffer,
               bool                             reuse_query_order = false);

    template <typename NodePred, typename LeafPred>
    void launch_query(cuda_tool::CBufferView<AABB>     query_aabbs,
                      cuda_tool::CBufferView<IndexT>   query_BIDs,
                      cuda_tool::CBufferView<IndexT>   query_CIDs,
                      cuda_tool::CBuffer2DView<IndexT> cmts,
                      NodePred                         np,
                      LeafPred                         lp,
                      QueryBuffer&                     qbuffer,
                      bool                             rebuild_query = true,
                      bool         reuse_query_order                 = false,
                      cudaStream_t stream = cuda_tool::default_stream());

    // Publish a device-produced count and grow the output if a retry is
    // required. The caller relaunches the same query when this returns true.
    bool prepare_query_result(QueryBuffer& qbuffer, int count);

    Config&       config() noexcept { return m_impl.config; }
    const Config& config() const noexcept { return m_impl.config; }

  public:
    class Impl
    {
      public:
        static void calcMaxBVFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                     cuda_tool::VarView<AABB>     scene_box,
                                     cudaStream_t stream = cuda_tool::default_stream());
        static void calcMCsFromBox(cuda_tool::CBufferView<AABB>    aabbs,
                                   cuda_tool::CVarView<AABB>       scene_box,
                                   cuda_tool::BufferView<uint32_t> codes,
                                   cudaStream_t stream = cuda_tool::default_stream());
        void calcInverseMapping(cudaStream_t stream = cuda_tool::default_stream());
        void buildPrimitivesFromBox(cuda_tool::CBufferView<AABB> aabbs,
                                    cudaStream_t stream = cuda_tool::default_stream());
        void calcExtNodeSplitMetrics(cudaStream_t stream = cuda_tool::default_stream());
        void buildIntNodes(int size, cudaStream_t stream = cuda_tool::default_stream());
        void calcIntNodeOrders(int size, cudaStream_t stream = cuda_tool::default_stream());
        void updateBvhExtNodeLinks(int size, cudaStream_t stream = cuda_tool::default_stream());
        void reorderNode(int intSize, cudaStream_t stream = cuda_tool::default_stream());
        void propagateInformativeMetadata(int intSize);
        void build(cuda_tool::CBufferView<AABB>   aabbs,
                   cuda_tool::CBufferView<IndexT> bids,
                   cuda_tool::CBufferView<IndexT> cids,
                   cudaStream_t stream = cuda_tool::default_stream());
        bool refit(cuda_tool::CBufferView<AABB>   aabbs,
                   cuda_tool::CBufferView<IndexT> bids,
                   cuda_tool::CBufferView<IndexT> cids,
                   cudaStream_t stream = cuda_tool::default_stream());

        // Pre-loads query bid/cid into shared memory before the traversal loop.
        // node_cull receives NodePredInfo with query_bid/query_cid from SMem.
        template <typename NodeCull, typename PairPred>
        void stacklessSelf(NodeCull                        node_cull,
                           PairPred                        pair_pred,
                           cuda_tool::VarView<int>         cpNum,
                           cuda_tool::BufferView<Vector2i> buffer,
                           cudaStream_t stream = cuda_tool::default_stream());

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
                            cuda_tool::BufferView<Vector2i> buffer,
                            cudaStream_t stream = cuda_tool::default_stream());

        cuda_tool::CBufferView<AABB>      objs;
        cuda_tool::CBufferView<IndexT>    bids;
        cuda_tool::CBufferView<IndexT>    cids;
        cuda_tool::DeviceVar<AABB>        scene_box;
        cuda_tool::DeviceVector<uint32_t> flags;
        cuda_tool::DeviceVector<uint32_t> mtcode;
        cuda_tool::DeviceVector<uint32_t> sorted_mtcode;
        cuda_tool::DeviceVector<int32_t>  sorted_id;
        cuda_tool::DeviceVector<int32_t>  primMap;
        cuda_tool::DeviceVector<int>      metric;
        cuda_tool::DeviceVector<uint32_t> count;
        cuda_tool::DeviceVector<int>      tkMap;
        cuda_tool::DeviceVector<uint32_t> offsetTable;
        cuda_tool::DeviceVector<AABB>     ext_aabb;
        cuda_tool::DeviceVector<int>      ext_idx;
        cuda_tool::DeviceVector<int>      ext_lca;
        cuda_tool::DeviceVector<uint32_t> ext_par;
        cuda_tool::DeviceVector<int>      int_lc;
        cuda_tool::DeviceVector<int>      int_rc;
        cuda_tool::DeviceVector<int>      int_par;
        cuda_tool::DeviceVector<int>      int_range_x;
        cuda_tool::DeviceVector<int>      int_range_y;
        // Maximum Morton-sorted leaf rank for each reordered internal node.
        // stacklessSelf uses it to skip the already-covered half-tree.
        cuda_tool::DeviceVector<int> self_max_rank;
        // Stable reordered-tree links and per-refit publication state. The
        // last arriving child publishes each parent before walking upward.
        cuda_tool::DeviceVector<int>      refit_parent;
        cuda_tool::DeviceVector<int>      refit_right_child;
        cuda_tool::DeviceVector<int>      refit_arrivals;
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
