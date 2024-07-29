#pragma once
#include <sim_system.h>
#include <collision_detection/global_dcd_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <muda/buffer/device_buffer.h>

namespace uipc::backend::cuda
{
class SimplexDCDFilter : public SimSystem
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

        muda::CBufferView<Vector3>  positions() const noexcept;
        muda::CBufferView<IndexT>   surf_vertices() const noexcept;
        muda::CBufferView<Vector2i> surf_edges() const noexcept;
        muda::CBufferView<Vector3i> surf_triangles() const noexcept;

        void PTs(muda::CBufferView<Vector4i> PTs) noexcept;
        void EEs(muda::CBufferView<Vector4i> EEs) noexcept;
        void PEs(muda::CBufferView<Vector3i> PEs) noexcept;
        void PPs(muda::CBufferView<Vector2i> PPs) noexcept;


      private:
        friend class GlobalDCDFilter;
        Impl* m_impl;
    };

    class Impl
    {
      public:
        void record_friction_candidates();

        GlobalVertexManager* global_vertex_manager = nullptr;
        GlobalSimpicialSurfaceManager* global_simplicial_surface_manager = nullptr;
        GlobalContactManager* global_contact_manager = nullptr;

        muda::CBufferView<Vector4i> PTs;
        muda::CBufferView<Vector4i> EEs;
        muda::CBufferView<Vector3i> PEs;
        muda::CBufferView<Vector2i> PPs;

        muda::DeviceBuffer<Vector4i> friction_PT;
        muda::DeviceBuffer<Vector4i> friction_EE;
        muda::DeviceBuffer<Vector3i> friction_PE;
        muda::DeviceBuffer<Vector2i> friction_PP;

        Float reserve_ratio = 1.1;

        template <typename T>
        void loose_resize(muda::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve(size * reserve_ratio);
            }
            buffer.resize(size);
        }
    };

    muda::CBufferView<Vector4i> PTs() const noexcept;
    muda::CBufferView<Vector4i> EEs() const noexcept;
    muda::CBufferView<Vector3i> PEs() const noexcept;
    muda::CBufferView<Vector2i> PPs() const noexcept;

    muda::CBufferView<Vector4i> friction_PTs() const noexcept;
    muda::CBufferView<Vector4i> friction_EEs() const noexcept;
    muda::CBufferView<Vector3i> friction_PEs() const noexcept;
    muda::CBufferView<Vector2i> friction_PPs() const noexcept;

  protected:
    virtual void do_detect(FilterInfo& info) = 0;

  private:
    friend class GlobalDCDFilter;
    Impl         m_impl;
    void         detect();
    void         record_friction_candidates();
    virtual void do_build() override final;
};
}  // namespace uipc::backend::cuda