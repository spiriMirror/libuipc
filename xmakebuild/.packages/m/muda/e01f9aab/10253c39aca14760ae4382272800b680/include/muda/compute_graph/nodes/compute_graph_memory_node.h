#pragma once
#include <muda/compute_graph/compute_graph.h>
#include <muda/compute_graph/compute_graph_node.h>
#include <muda/graph/graph.h>
#include <muda/graph/memory_node.h>

namespace muda
{
using ComputeGraphMemcpyNode =
    ComputeGraphNode<MemcpyNode, ComputeGraphNodeType::MemcpyNode>;

using ComputeGraphMemsetNode =
    ComputeGraphNode<MemsetNode, ComputeGraphNodeType::MemsetNode>;
}  // namespace muda
