#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>

namespace uipc::backend::cuda
{
class VertexReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    using VertexCountInfo        = GlobalVertexManager::VertexCountInfo;
    using VertexAttributeInfo    = GlobalVertexManager::VertexAttributeInfo;
    using VertexDisplacementInfo = GlobalVertexManager::VertexDisplacementInfo;

    class BuildInfo
    {
      public:
    };

    IndexT vertex_offset() const noexcept { return m_vertex_offset; }
    IndexT vertex_count() const noexcept { return m_vertex_count; }

  protected:
    virtual void do_build(BuildInfo& info)                           = 0;
    virtual void do_report_count(VertexCountInfo& vertex_count_info) = 0;
    virtual void do_report_attributes(VertexAttributeInfo& vertex_attribute_info) = 0;
    virtual void do_report_displacements(VertexDisplacementInfo& vertex_displacement_info) = 0;

  private:
    friend class GlobalVertexManager;
    virtual void do_build() final override;
    void         report_count(VertexCountInfo& vertex_count_info);
    void         report_attributes(VertexAttributeInfo& vertex_attribute_info);
    void report_displacements(VertexDisplacementInfo& vertex_displacement_info);

    SizeT  m_index         = ~0ull;
    IndexT m_vertex_offset = -1;
    IndexT m_vertex_count  = -1;
};
}  // namespace uipc::backend::cuda
