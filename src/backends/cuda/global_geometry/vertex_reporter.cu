#include <global_geometry/vertex_reporter.h>

namespace uipc::backend::cuda
{
void VertexReporter::do_build()
{
    auto& global_vertex_manager = require<GlobalVertexManager>();

    BuildInfo info;
    do_build(info);

    global_vertex_manager.add_reporter(this);
}

void VertexReporter::report_count(GlobalVertexManager::VertexCountInfo& info)
{
    do_report_count(info);
    // Record the vertex count
    m_vertex_count = info.m_count;
}

void VertexReporter::report_attributes(GlobalVertexManager::VertexAttributeInfo& info)
{
    do_report_attributes(info);
    // Record the global vertex info
    m_vertex_offset = info.coindices().offset();
    UIPC_ASSERT(m_vertex_count == info.coindices().size(),
                "Vertex count mismatch: expected {}, got {}",
                m_vertex_count,
                info.coindices().size());
    m_vertex_count = info.coindices().size();
}

void VertexReporter::report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
    do_report_displacements(info);
}
}  // namespace uipc::backend::cuda
