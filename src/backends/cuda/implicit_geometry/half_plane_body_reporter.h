#pragma once
#include <global_geometry/body_reporter.h>
#include <implicit_geometry/half_plane.h>

namespace uipc::backend::cuda
{
class HalfPlaneBodyReporter final : public BodyReporter
{
  public:
    using BodyReporter::BodyReporter;

    class Impl;

    class Impl
    {
      public:
        HalfPlane* half_plane = nullptr;

        void report_count(BodyCountInfo& info);
        void report_attributes(BodyAttributeInfo& info);
    };

  private:
    virtual void do_build(BuildInfo& info) override;

    virtual void do_init(InitInfo& info) override;
    void         do_report_count(BodyCountInfo& info) override;
    void         do_report_attributes(BodyAttributeInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda