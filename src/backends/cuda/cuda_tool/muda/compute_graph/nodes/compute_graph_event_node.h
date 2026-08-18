#pragma once
#include <muda/compute_graph/compute_graph.h>
#include <muda/compute_graph/compute_graph_node.h>
#include <muda/graph/graph.h>
#include <muda/graph/event_node.h>

namespace muda
{
using ComputeGraphEventRecordNode =
    ComputeGraphNode<EventRecordNode, ComputeGraphNodeType::EventRecordNode>;
using ComputeGraphEventWaitNode =
    ComputeGraphNode<EventWaitNode, ComputeGraphNodeType::EventWaitNode>;
}  // namespace muda