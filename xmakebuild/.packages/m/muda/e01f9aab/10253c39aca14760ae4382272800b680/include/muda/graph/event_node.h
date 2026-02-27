#pragma once
#include <muda/graph/graph_base.h>

namespace muda
{
class EventRecordNode : public GraphNode
{
  public:
    using this_type = EventRecordNode;
    friend class Graph;
};

class EventWaitNode : public GraphNode
{
  public:
    using this_type = EventWaitNode;
    friend class Graph;
};
}  // namespace muda