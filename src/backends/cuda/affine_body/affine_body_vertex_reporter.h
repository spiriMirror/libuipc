#pragma once
#include <global_geometry/vertex_reporter.h>
#include <affine_body/affine_body_dynamics.h>
namespace uipc::backend::cuda
{
class AffineBodyBodyReporter;
class AffineBodyVertexReporter final : public VertexReporter
{
  public:
    using VertexReporter::VertexReporter;

    class Impl
    {
      public:
        void report_count(VertexCountInfo& info);

        void init_attributes(VertexAttributeInfo& info);
        void update_attributes(VertexAttributeInfo& info);

        void report_displacements(VertexDisplacementInfo& info);

        AffineBodyDynamics*       affine_body_dynamics;
        AffineBodyBodyReporter*   body_reporter = nullptr;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }

        bool require_update_attributes = false;
    };

    // Request to update vertex attributes before next simulation step
    void request_attribute_update() noexcept;

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_report_count(VertexCountInfo& info) override;
    virtual void do_report_attributes(VertexAttributeInfo& info) override;
    virtual void do_report_displacements(VertexDisplacementInfo& info) override;
    virtual U64 get_uid() const noexcept override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
