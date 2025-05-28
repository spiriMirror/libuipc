#pragma once
#include <global_geometry/body_reporter.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class AffineBodyBodyReporter final : public BodyReporter
{
  public:
    using BodyReporter::BodyReporter;

    class Impl
    {
      public:
        void report_count(BodyCountInfo& info);
        void report_attributes(BodyAttributeInfo& info);

        AffineBodyDynamics* affine_body_dynamics = nullptr;
    };

  private:
    virtual void do_build(BuildInfo& info) override;

    virtual void do_init(InitInfo& info) override;
    virtual void do_report_count(BodyCountInfo& info) override;
    virtual void do_report_attributes(BodyAttributeInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda