#pragma once
#include <global_geometry/vertex_reporter.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
class FiniteElementBodyReporter;
class FiniteElementVertexReporter final : public VertexReporter
{
  public:
    using VertexReporter::VertexReporter;

    class Impl
    {
      public:
        void report_count(VertexCountInfo& info);
        void report_attributes(VertexAttributeInfo& info);
        void report_displacements(VertexDisplacementInfo& info);

        FiniteElementMethod*       finite_element_method;
        FiniteElementMethod::Impl& fem()
        {
            return finite_element_method->m_impl;
        }
        FiniteElementBodyReporter* body_reporter = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_report_count(VertexCountInfo& info) override;
    virtual void do_report_attributes(VertexAttributeInfo& info) override;
    virtual void do_report_displacements(VertexDisplacementInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
