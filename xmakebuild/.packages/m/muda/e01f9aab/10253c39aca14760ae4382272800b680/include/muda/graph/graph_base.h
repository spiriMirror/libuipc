#pragma once
#include <muda/tools/version.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_device_runtime_api.h>

#include <list>
#include <vector>
#include <memory>
#include <functional>

#include "../check/check_cuda_errors.h"

namespace muda
{
class Graph;
class GraphExec;

class NodeParms
{
  public:
    NodeParms()          = default;
    virtual ~NodeParms() = default;
};

class GraphNode
{
  protected:
    cudaGraphNode_t m_handle;

  public:
    friend class GraphExec;
    GraphNode()
        : m_handle(nullptr)
    {
    }
    using this_type = GraphNode;
    friend class Graph;
    cudaGraphNode_t handle() const { return m_handle; }
};
}  // namespace muda