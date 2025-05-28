#pragma once
#include <sim_system.h>
#include <global_geometry/simplicial_surface_reporter.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_vertex_reporter.h>

namespace uipc::backend::cuda
{
class FiniteElementSurfaceReporter : public SimplicialSurfaceReporter
{
  public:
    using SimplicialSurfaceReporter::SimplicialSurfaceReporter;

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

    class Impl
    {
      public:
        void init(backend::WorldVisitor& world);

        void report_count(backend::WorldVisitor& world, SurfaceCountInfo& info);
        void report_attributes(backend::WorldVisitor& world, SurfaceAttributeInfo& info);

        FiniteElementMethod*       finite_element_method = nullptr;
        FiniteElementMethod::Impl& fem()
        {
            return finite_element_method->m_impl;
        }
        FiniteElementVertexReporter* finite_element_vertex_reporter = nullptr;

        // core invariant data
        vector<SurfaceInfo> body_surface_infos;

        vector<SizeT> geo_surf_vertex_offsets;
        vector<SizeT> geo_surf_vertex_counts;
        vector<SizeT> geo_surf_edge_offsets;
        vector<SizeT> geo_surf_edge_counts;
        vector<SizeT> geo_surf_triangle_offsets;
        vector<SizeT> geo_surf_triangle_counts;

        vector<IndexT>   surf_vertices;
        vector<Vector2i> surf_edges;
        vector<Vector3i> surf_triangles;
    };

  protected:
    virtual void do_build(BuildInfo& info) override;

    virtual void do_init(SurfaceInitInfo& info) override;
    virtual void do_report_count(SurfaceCountInfo& info) override;
    virtual void do_report_attributes(SurfaceAttributeInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
