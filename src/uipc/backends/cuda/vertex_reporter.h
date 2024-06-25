#pragma once
#include <sim_system.h>
#include <global_vertex_manager.h>

namespace uipc::backend::cuda
{
class VertexReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_build() override;
    virtual void do_report_count(GlobalVertexManager::VertexCountInfo& vertex_count_info) = 0;
    virtual void do_report_attributes(GlobalVertexManager::VertexAttributeInfo& vertex_attribute_info) = 0;
    virtual void do_report_displacements(
        GlobalVertexManager::VertexDisplacementInfo& vertex_displacement_info) = 0;

  private:
    friend class GlobalVertexManager;
    void report_count(GlobalVertexManager::VertexCountInfo& vertex_count_info);
    void report_attributes(GlobalVertexManager::VertexAttributeInfo& vertex_attribute_info);
    void report_displacements(GlobalVertexManager::VertexDisplacementInfo& vertex_displacement_info);

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
