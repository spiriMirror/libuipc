#pragma once
#include <map>
#include <string>
#include <muda/compute_graph/compute_graph_node_type.h>
#include <muda/compute_graph/compute_graph_node_id.h>
#include <muda/compute_graph/compute_graph_var_usage.h>
#include <muda/compute_graph/compute_graph_var_id.h>
#include <muda/compute_graph/compute_graph_fwd.h>

namespace muda
{
class ComputeGraphNodeBase
{
  public:
    auto node_id() const { return m_node_id; }
    auto access_index() const { return m_access_index; }
    auto type() const { return m_type; }
    auto name() const { return std::string_view{m_name}; }

    virtual ~ComputeGraphNodeBase() = default;

  protected:
    template <typename T>
    using S = std::shared_ptr<T>;

    friend class ComputeGraph;
    friend class ComputeGraphVarBase;
    ComputeGraphNodeBase(std::string_view name, NodeId node_id, uint64_t access_index, ComputeGraphNodeType type)
        : m_name(name)
        , m_node_id(node_id)
        , m_access_index(access_index)
        , m_type(type)
    {
    }

    std::string m_name;
    NodeId      m_node_id;
    uint64_t    m_access_index;

    ComputeGraphNodeType m_type;
    cudaGraphNode_t      m_cuda_node = nullptr;


    auto handle() const { return m_cuda_node; }
    void set_handle(cudaGraphNode_t handle) { m_cuda_node = handle; }
    auto is_valid() const { return m_cuda_node; }
};

template <typename NodeT, ComputeGraphNodeType Type>
class ComputeGraphNode : public ComputeGraphNodeBase
{
  protected:
    friend class ComputeGraph;
    friend class details::ComputeGraphAccessor;
    ComputeGraphNode(NodeId node_id, uint64_t access_graph_index);

    S<NodeT> m_node;
    void     set_node(S<NodeT> node);
    virtual ~ComputeGraphNode() = default;
};
}  // namespace muda

#include "details/compute_graph_node.inl"