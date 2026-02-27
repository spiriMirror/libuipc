#pragma once
#include <muda/launch/launch_base.h>

namespace muda
{
class GraphViewer;
class GraphLaunch : public LaunchBase<GraphLaunch>
{
  public:
    GraphLaunch(cudaStream_t stream = nullptr)
        : LaunchBase(stream)
    {
    }
    ~GraphLaunch() = default;

    GraphLaunch& launch(const GraphViewer& graph);
};
}  // namespace muda

#include "details/graph_launch.inl"