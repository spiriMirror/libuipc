#pragma once
#include <sim_system.h>
#include <surface_reporter.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_vertex_reporter.h>
namespace uipc::backend::cuda
{
class AffinebodySurfaceReporter : public SurfaceReporter
{
  public:
    using SurfaceReporter::SurfaceReporter;

    class Impl
    {
      public:
        void init_surface_on_host(backend::WorldVisitor&);

        AffineBodyDynamics*       affine_body_dynamics        = nullptr;
        AffineBodyVertexReporter* affine_body_vertex_reporter = nullptr;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }

        vector<IndexT>   surf_vertices;
        vector<Vector2i> surf_edges;
        vector<Vector3i> surf_triangles;
    };

  protected:
    virtual void do_build() override;
    virtual void do_report_count(GlobalSurfaceManager::SurfaceCountInfo& info) override;
    virtual void do_report_attributes(GlobalSurfaceManager::SurfaceAttributeInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
