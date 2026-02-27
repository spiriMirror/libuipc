#include <muda/graph/graph_viewer.h>

namespace muda
{
MUDA_INLINE GraphLaunch& GraphLaunch::launch(const GraphViewer& graph)
{
	graph.launch();
    return *this;
}
}  // namespace muda