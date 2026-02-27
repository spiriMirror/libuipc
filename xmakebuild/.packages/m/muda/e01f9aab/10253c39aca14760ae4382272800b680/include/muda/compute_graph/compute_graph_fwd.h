#pragma once
#include <muda/compute_graph/compute_graph_var_id.h>
#include <muda/compute_graph/compute_graph_closure_id.h>
#include <muda/compute_graph/compute_graph_node_id.h>
#include <muda/compute_graph/compute_graph_var_usage.h>
namespace muda
{
class ComputeGraphVarBase;
class ComputeGraphNodeBase;
class ComputeGraphVarManager;
class ComputeGraphClosure;
template <typename T>
class ComputeGraphVar;
class ComputeGraph;
class ComputeGraphGraphvizOptions;
namespace details
{
    class ComputeGraphAccessor;
}
}  // namespace muda