#pragma once
#include <muda/compute_graph/compute_graph_closure_id.h>
namespace muda
{
class ComputeGraphDependency
{
  public:
    ClosureId from;
    ClosureId to;
};
}  // namespace muda