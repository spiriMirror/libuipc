#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <muda/buffer/buffer_view.h>

namespace uipc::backend::cuda
{
class SimplicialSurfaceReporter;
class GlobalSimpicialSurfaceManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

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

    class SurfaceCountInfo
    {
      public:
        void surf_vertex_count(SizeT count) noexcept;
        void surf_edge_count(SizeT count) noexcept;
        void surf_triangle_count(SizeT count) noexcept;
        void changable(bool value) noexcept;

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
        SurfaceAttributeInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        muda::BufferView<IndexT>   surf_vertices() noexcept;
        muda::BufferView<Vector2i> surf_edges() noexcept;
        muda::BufferView<Vector3i> surf_triangles() noexcept;
        const ReporterInfo&        reporter_info() const noexcept;

      private:
        friend class Impl;
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
    };

    class SurfaceInitInfo
    {
      public:
    };


    class Impl
    {
      public:
        void init();
        void _collect_codim_vertices();

        // core invariant data
        vector<ReporterInfo> reporter_infos;
        // related data
        SizeT total_surf_vertex_count   = 0;
        SizeT total_surf_edge_count     = 0;
        SizeT total_surf_triangle_count = 0;

        muda::DeviceBuffer<IndexT>   codim_vertices;
        muda::DeviceBuffer<IndexT>   surf_vertices;
        muda::DeviceBuffer<IndexT>   codim_vertex_flags;
        muda::DeviceBuffer<Vector2i> surf_edges;
        muda::DeviceBuffer<Vector3i> surf_triangles;

        GlobalVertexManager* global_vertex_manager = nullptr;
        SimSystemSlotCollection<SimplicialSurfaceReporter> reporters;
        muda::DeviceVar<int> selected_codim_0d_count;
    };

    muda::CBufferView<IndexT>   codim_vertices() const noexcept;
    muda::CBufferView<IndexT>   surf_vertices() const noexcept;
    muda::CBufferView<Vector2i> surf_edges() const noexcept;
    muda::CBufferView<Vector3i> surf_triangles() const noexcept;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    Impl m_impl;
    void init();     // only called by SimEngine
    void rebuild();  // only called by SimEngine

    friend class SimplicialSurfaceReporter;
    void add_reporter(SimplicialSurfaceReporter* reporter) noexcept; // only called by SimplicialSurfaceReporter
};
}  // namespace uipc::backend::cuda
