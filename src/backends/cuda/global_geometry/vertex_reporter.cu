#include <global_geometry/vertex_reporter.h>

namespace uipc::backend::cuda
{
void VertexReporter::do_build() {}

void VertexReporter::report_count(GlobalVertexManager::VertexCountInfo& vertex_count_info)
{
    do_report_count(vertex_count_info);
}

void VertexReporter::report_attributes(GlobalVertexManager::VertexAttributeInfo& vertex_attribute_info)
{
    do_report_attributes(vertex_attribute_info);
}

void VertexReporter::report_displacements(GlobalVertexManager::VertexDisplacementInfo& vertex_displacement_info)
{
    do_report_displacements(vertex_displacement_info);
}
}  // namespace uipc::backend::cuda
