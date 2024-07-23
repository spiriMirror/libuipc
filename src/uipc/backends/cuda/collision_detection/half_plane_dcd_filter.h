#pragma once
#include <sim_system.h>
#include <collision_detection/global_dcd_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <muda/buffer/device_buffer.h>
#include <implicit_geometry/half_plane.h>

namespace uipc::backend::cuda
{
class HalfPlaneDCDFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class FilterInfo
    {
      public:
        FilterInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        Float d_hat() const noexcept;

        muda::CBufferView<Vector3> plane_normals() const noexcept;
        muda::CBufferView<Vector3> plane_positions() const noexcept;

        muda::CBufferView<Vector3> positions() const noexcept;
        muda::CBufferView<IndexT>  surf_vertices() const noexcept;

        void PHs(muda::CBufferView<Vector2i> Ps) noexcept;

      private:
        friend class GlobalDCDFilter;
        Impl* m_impl;
    };

    class BuildInfo
    {
      public:
    };

    class Impl
    {
      public:
        GlobalVertexManager* global_vertex_manager = nullptr;
        GlobalSimpicialSurfaceManager* global_simplicial_surface_manager = nullptr;
        GlobalContactManager* global_contact_manager = nullptr;
        HalfPlane*            half_plane             = nullptr;

        muda::CBufferView<Vector2i> PHs;
    };

    muda::CBufferView<Vector2i> PHs() noexcept;

  protected:
    virtual void do_detect(FilterInfo& info) = 0;
    virtual void do_build(BuildInfo& info){};

  private:
    friend class GlobalDCDFilter;
    Impl         m_impl;
    void         detect();
    virtual void do_build() override final;
};
}  // namespace uipc::backend::cuda