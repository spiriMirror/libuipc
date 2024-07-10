#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <muda/buffer/buffer_view.h>

namespace uipc::backend::cuda
{
class SurfaceReporter;
class GlobalSimpicialSurfaceManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    void add_reporter(SurfaceReporter* reporter) noexcept;


    class SurfaceCountInfo
    {
      public:
        void surf_vertex_count(SizeT count) noexcept
        {
            m_surf_vertex_count = count;
        }
        void surf_edge_count(SizeT count) noexcept
        {
            m_surf_edge_count = count;
        }
        void surf_triangle_count(SizeT count) noexcept
        {
            m_surf_triangle_count = count;
        }

        void changable(bool value) { m_changable = value; }

      private:
        friend class Impl;
        SizeT m_surf_vertex_count   = 0;
        SizeT m_surf_edge_count     = 0;
        SizeT m_surf_triangle_count = 0;
        bool  m_changable           = false;
    };

    class SurfaceAttributeInfo
    {
      public:
        SurfaceAttributeInfo(Impl* impl)
            : m_impl(impl)
        {
        }
        muda::BufferView<IndexT> surf_vertices() noexcept
        {
            return m_surf_vertices;
        }

        muda::BufferView<Vector2i> surf_edges() noexcept
        {
            return m_surf_edges;
        }
        muda::BufferView<Vector3i> surf_triangles() noexcept
        {
            return m_surf_triangles;
        }

      private:
        friend class Impl;
        muda::BufferView<IndexT>   m_surf_vertices;
        muda::BufferView<Vector2i> m_surf_edges;
        muda::BufferView<Vector3i> m_surf_triangles;
        Impl*                      m_impl = nullptr;
    };

    class ReporterInfo
    {
      public:
        SizeT surf_vertex_offset   = 0;
        SizeT surf_vertex_count    = 0;
        SizeT surf_edge_offset     = 0;
        SizeT surf_edge_count      = 0;
        SizeT surf_triangle_offset = 0;
        SizeT surf_triangle_count  = 0;
    };

    class Impl
    {
      public:
        void init_surface_info();

        // core invariant data
        vector<ReporterInfo> reporter_infos;
        // related data
        SizeT total_surf_vertex_count   = 0;
        SizeT total_surf_edge_count     = 0;
        SizeT total_surf_triangle_count = 0;


        muda::DeviceBuffer<IndexT>   surf_vertices;
        muda::DeviceBuffer<Vector2i> surf_edges;
        muda::DeviceBuffer<Vector3i> surf_triangles;

        GlobalVertexManager* global_vertex_manager = nullptr;
        SimSystemSlotCollection<SurfaceReporter> reporters;
    };

    muda::CBufferView<IndexT>   surf_vertices() const noexcept;
    muda::CBufferView<Vector2i> surf_edges() const noexcept;
    muda::CBufferView<Vector3i> surf_triangles() const noexcept;


  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    Impl m_impl;
    void init_surface_info();
    void rebuild_surface_info();
};
}  // namespace uipc::backend::cuda
