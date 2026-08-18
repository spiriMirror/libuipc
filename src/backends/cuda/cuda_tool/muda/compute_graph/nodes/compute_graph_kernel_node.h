#pragma once
#include <muda/compute_graph/compute_graph_node.h>
#include <muda/graph/graph.h>
#include <muda/graph/kernel_node.h>

namespace muda
{
using ComputeGraphKernelNode =
    ComputeGraphNode<KernelNode, ComputeGraphNodeType::KernelNode>;
}  // namespace muda