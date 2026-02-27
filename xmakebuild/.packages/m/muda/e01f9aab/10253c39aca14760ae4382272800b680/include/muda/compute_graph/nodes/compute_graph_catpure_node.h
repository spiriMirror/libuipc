#pragma once
#include <muda/compute_graph/compute_graph_node.h>
#include <muda/graph/graph.h>

namespace muda
{
class ComputeGraphCaptureNode : public ComputeGraphNodeBase
{
  protected:
    friend class ComputeGraph;
    friend class details::ComputeGraphAccessor;
    ComputeGraphCaptureNode(NodeId node_id, uint64_t access_index)
        : ComputeGraphNodeBase(enum_name(ComputeGraphNodeType::CaptureNode),
                               node_id,
                               access_index,
                               ComputeGraphNodeType::CaptureNode)
    {
        auto n = std::string_view{
            details::LaunchInfoCache::current_capture_name().auto_select()};
        if(n.empty() || n == "")
            m_name += std::string(":~");
        else
            m_name += std::string(":") + std::string(n.data());
    }

    virtual ~ComputeGraphCaptureNode() override { update_sub_graph(nullptr); }

    void set_node(cudaGraphNode_t node) { set_handle(node); }

    void update_sub_graph(cudaGraph_t sub_graph)
    {
        if(m_sub_graph)
            checkCudaErrors(cudaGraphDestroy(m_sub_graph));
        m_sub_graph = sub_graph;
    }

    cudaGraph_t m_sub_graph = nullptr;
};
}  // namespace muda
