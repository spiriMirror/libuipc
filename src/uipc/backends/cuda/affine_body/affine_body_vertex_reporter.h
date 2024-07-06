#pragma once
#include <global_geometry/vertex_reporter.h>
#include <affine_body/affine_body_dynamics.h>
namespace uipc::backend::cuda
{
class AffineBodyVertexReporter : public VertexReporter
{
  public:
    using VertexReporter::VertexReporter;

    class Impl
    {
      public:
        void report_count(GlobalVertexManager::VertexCountInfo& info);
        void report_attributes(GlobalVertexManager::VertexAttributeInfo& info);
        void report_displacements(GlobalVertexManager::VertexDisplacementInfo& info);

        SizeT                     reporter_vertex_offset = 0;
        SizeT                     reporter_vertex_count  = 0;
        AffineBodyDynamics*       affine_body_dynamics;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }
    };

    SizeT vertex_offset() const noexcept
    {
        return m_impl.reporter_vertex_offset;
    }
    SizeT vertex_count() const noexcept { return m_impl.reporter_vertex_count; }

  protected:
    virtual void do_build() override;

    virtual void do_report_count(GlobalVertexManager::VertexCountInfo& info) override;
    virtual void do_report_attributes(GlobalVertexManager::VertexAttributeInfo& info) override;
    virtual void do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
