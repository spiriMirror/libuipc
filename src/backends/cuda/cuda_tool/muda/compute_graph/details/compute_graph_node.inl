#include <muda/compute_graph/compute_graph_var_manager.h>
#include <muda/tools/launch_info_cache.h>
namespace muda
{
template <typename NodeT, ComputeGraphNodeType Type>
MUDA_INLINE ComputeGraphNode<NodeT, Type>::ComputeGraphNode(NodeId node_id, uint64_t access_graph_index)
    : ComputeGraphNodeBase(enum_name(Type), node_id, access_graph_index, Type)
{
    if constexpr(Type == ComputeGraphNodeType::KernelNode)
    {
        auto n = std::string_view{
            details::LaunchInfoCache::current_kernel_name().auto_select()};
        if(n.empty() || n == "")
            m_name += std::string(":~");
        else
            m_name += std::string(":") + std::string(n.data());
    }
}

template <typename NodeT, ComputeGraphNodeType Type>
MUDA_INLINE void ComputeGraphNode<NodeT, Type>::set_node(S<NodeT> node)
{
    m_node = node;
    set_handle(m_node->handle());
}
}  // namespace muda