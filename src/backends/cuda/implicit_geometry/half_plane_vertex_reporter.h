#pragma once
#include <global_geometry/vertex_reporter.h>

namespace uipc::backend::cuda
{
class HalfPlane;
class HalfPlaneVertexReporter : public VertexReporter
{
  public:
    using VertexReporter::VertexReporter;

    class Impl;

    class Impl
    {
      public:
        HalfPlane* half_plane = nullptr;

        void report_count(GlobalVertexManager::VertexCountInfo& info);
        void report_attributes(GlobalVertexManager::VertexAttributeInfo& info);
        void report_displacements(GlobalVertexManager::VertexDisplacementInfo& info);

        size_t vertex_global_offset = ~0ull;
        size_t vertex_count         = 0;
    };

    size_t vertex_global_offset() const;
    size_t vertex_count() const;

  protected:
    virtual void do_build(BuildInfo& info) override;
    void do_report_count(GlobalVertexManager::VertexCountInfo& vertex_count_info) override;
    void do_report_attributes(GlobalVertexManager::VertexAttributeInfo& vertex_attribute_info) override;
    void do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& vertex_displacement_info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
