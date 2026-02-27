#pragma once
#include <cuda.h>
#include <muda/viewer/viewer_base.h>
#include <muda/graph/graph_instantiate_flag.h>
namespace muda
{
class GraphViewer : public ViewerBase<true>
{
    MUDA_VIEWER_COMMON_NAME(GraphViewer);

  public:
    MUDA_GENERIC GraphViewer() = default;
    MUDA_GENERIC GraphViewer(cudaGraphExec_t graph, Flags<GraphInstantiateFlagBit> flags);

    MUDA_GENERIC void launch(cudaStream_t stream = nullptr) const;

    MUDA_DEVICE void tail_launch() const;
    MUDA_DEVICE void fire_and_forget() const;

    MUDA_GENERIC auto handle() const { return m_graph; }

  private:
    friend class ComputeGraph;
    cudaGraphExec_t                m_graph = nullptr;
    Flags<GraphInstantiateFlagBit> m_flags;
};

template <>
struct read_only_view<GraphViewer>
{
    using type = GraphViewer;
};

template <>
struct read_write_view<GraphViewer>
{
    using type = GraphViewer;
};

static_assert(is_uniform_view_v<GraphViewer>);
}  // namespace muda

#include "details/graph_viewer.inl"