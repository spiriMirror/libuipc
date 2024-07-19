#pragma once
#include <global_geometry/vertex_reporter.h>

namespace uipc::backend::cuda
{
class HalpPlaneVertexReporter : public VertexReporter
{
  public:
    using VertexReporter::VertexReporter;

    class Impl;

    class Impl
    {
    };

  protected:
    void do_report_count(GlobalVertexManager::VertexCountInfo& vertex_count_info) override;
    void do_report_attributes(GlobalVertexManager::VertexAttributeInfo& vertex_attribute_info) override;
    void do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& vertex_displacement_info) override;
};
}  // namespace uipc::backend::cuda
