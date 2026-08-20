#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
class SimplicialSurfaceReporter;
class GlobalSimplicialSurfaceManager : public SimSystem
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

        cuda_tool::BufferView<IndexT>   surf_vertices() noexcept;
        cuda_tool::BufferView<Vector2i> surf_edges() noexcept;
        cuda_tool::BufferView<Vector3i> surf_triangles() noexcept;
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

        cuda_tool::DeviceBuffer<IndexT>   codim_vertices;
        cuda_tool::DeviceBuffer<IndexT>   surf_vertices;
        cuda_tool::DeviceBuffer<IndexT>   codim_vertex_flags;
        cuda_tool::DeviceBuffer<Vector2i> surf_edges;
        cuda_tool::DeviceBuffer<Vector3i> surf_triangles;

        GlobalVertexManager* global_vertex_manager = nullptr;
        SimSystemSlotCollection<SimplicialSurfaceReporter> reporters;
        cuda_tool::DeviceVar<int> selected_codim_0d_count;
    };

    cuda_tool::CBufferView<IndexT>   codim_vertices() const noexcept;
    cuda_tool::CBufferView<IndexT>   surf_vertices() const noexcept;
    cuda_tool::CBufferView<Vector2i> surf_edges() const noexcept;
    cuda_tool::CBufferView<Vector3i> surf_triangles() const noexcept;

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
