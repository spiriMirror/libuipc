#pragma once
#include <sim_system.h>
#include <collision_detection/global_dcd_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <muda/buffer/device_buffer.h>

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

        muda::CBufferView<Vector3> positions() const noexcept;
        muda::CBufferView<IndexT>  surf_vertices() const noexcept;

        void Ps(muda::CBufferView<IndexT> Ps) noexcept;

      private:
        friend class GlobalDCDFilter;
        Impl* m_impl;
    };

    class Impl
    {
      public:
        GlobalVertexManager* global_vertex_manager = nullptr;
        GlobalSimpicialSurfaceManager* global_simplicial_surface_manager = nullptr;
        GlobalContactManager* global_contact_manager = nullptr;

        muda::CBufferView<IndexT> Ps;
    };

    muda::CBufferView<IndexT> Ps() noexcept;

  protected:
    virtual void do_detect(FilterInfo& info) = 0;

  private:
    friend class GlobalDCDFilter;
    Impl         m_impl;
    void         detect();
    virtual void do_build() override final;
};
}  // namespace uipc::backend::cuda