#pragma once

#include <sim_system.h>
#include <global_geometry/surface_reporter.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class AffinebodySurfaceReporter : public SurfaceReporter
{
  public:
    using SurfaceReporter::SurfaceReporter;

    class Impl;

    class SurfaceInfo
    {
      public:
        SizeT vertex_offset() const noexcept { return m_surf_vertex_offset; }
        SizeT vertex_count() const noexcept { return m_surf_vertex_count; }
        SizeT edge_offset() const noexcept { return m_surf_edge_offset; }
        SizeT edge_count() const noexcept { return m_surf_edge_count; }
        SizeT triangle_offset() const noexcept
        {
            return m_surf_triangle_offset;
        }
        SizeT triangle_count() const noexcept { return m_surf_triangle_count; }

      private:
        friend class Impl;
        SizeT m_surf_vertex_offset = 0;
        SizeT m_surf_vertex_count  = 0;

        SizeT m_surf_edge_offset = 0;
        SizeT m_surf_edge_count  = 0;

        SizeT m_surf_triangle_offset = 0;
        SizeT m_surf_triangle_count  = 0;
    };

    using BodySurfaceInfo = SurfaceInfo;

    class Impl
    {
      public:
        void init_surface(backend::WorldVisitor& world);
        void _init_geo_surface(backend::WorldVisitor& world);
        void _init_body_surface(backend::WorldVisitor& world);

        void report_count(backend::WorldVisitor&                  world,
                          GlobalSimpicialSurfaceManager::SurfaceCountInfo& info);
        void report_attributes(backend::WorldVisitor& world,
                               GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info);

        AffineBodyDynamics*       affine_body_dynamics = nullptr;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }

        // core invariant data
        vector<BodySurfaceInfo> body_surface_infos;
        // related data
        vector<SizeT> geo_surf_vertex_offsets;
        vector<SizeT> geo_surf_edges_offsets;
        vector<SizeT> geo_surf_triangles_offsets;
        vector<SizeT> geo_surf_vertex_counts;
        vector<SizeT> geo_surf_edges_counts;
        vector<SizeT> geo_surf_triangles_counts;

        vector<IndexT> geo_surf_vertices;
        vector<IndexT> geo_surf_edges;
        vector<IndexT> geo_surf_triangles;

        SizeT total_geo_surf_vertex_count   = 0;
        SizeT total_geo_surf_edge_count     = 0;
        SizeT total_geo_surf_triangle_count = 0;


        SizeT total_surf_vertex_count   = 0;
        SizeT total_surf_edge_count     = 0;
        SizeT total_surf_triangle_count = 0;


        vector<IndexT>   surf_vertices;
        vector<Vector2i> surf_edges;
        vector<Vector3i> surf_triangles;
    };

  protected:
    virtual void do_build() override;
    virtual void do_report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info) override;
    virtual void do_report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
