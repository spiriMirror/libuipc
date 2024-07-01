#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/ccd_filter.h>
#include <collision_detection/global_ccd_filter.h>

namespace uipc::backend::cuda
{
class SimplexCCDFilter : public CCDFilter
{
  public:
    using CCDFilter::CCDFilter;

    class Impl;

    class FilterInfo
    {
      public:
        FilterInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        Float                alpha() const noexcept { return m_alpha; }
        muda::VarView<Float> toi() const noexcept { return m_toi; }

        Float                       d_hat() const noexcept;
        muda::CBufferView<Vector3>  positions() const noexcept;
        muda::CBufferView<Vector3>  displacements() const noexcept;
        muda::CBufferView<IndexT>   surf_vertices() const noexcept;
        muda::CBufferView<Vector2i> surf_edges() const noexcept;
        muda::CBufferView<Vector3i> surf_triangles() const noexcept;

      private:
        friend class SimplexCCDFilter;
        Float                m_alpha = 0.0;
        muda::VarView<Float> m_toi;
        Impl*                m_impl = nullptr;
    };

    class Impl
    {
      public:
        GlobalVertexManager*           global_vertex_manager  = nullptr;
        GlobalSimpicialSurfaceManager* global_simplicial_surface_manager = nullptr;
        GlobalContactManager*          global_contact_manager = nullptr;
    };

  protected:
    virtual void do_filter_toi(FilterInfo& info) = 0;

  private:
    friend class GlobalCCDFilter;
    Impl m_impl;

    virtual void do_build() override final;

    void filter_toi(GlobalCCDFilter::FilterInfo& info) override final;
};
}  // namespace uipc::backend::cuda