#pragma once
#include <sim_system.h>
#include <global_vertex_manager.h>
#include <muda/buffer/buffer_view.h>

namespace uipc::backend::cuda
{
class GlobalSurfaceManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class Impl
    {
      public:
        GlobalVertexManager* global_vertex_manager = nullptr;

        muda::DeviceBuffer<IndexT>   surf_vertices;
        muda::DeviceBuffer<Vector2i> surf_edges;
        muda::DeviceBuffer<Vector3i> surf_triangles;
    };

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    Impl m_impl;
    void init_surface_info();
    void rebuild_surface_info();
};
}  // namespace uipc::backend::cuda
