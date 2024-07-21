#include <collision_detection/half_plane_ccd_filter.h>

namespace uipc::backend::cuda
{
void HalfPlaneCCDFilter::do_build()
{
    m_impl.global_vertex_manager = &require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = &require<GlobalSimpicialSurfaceManager>();
    m_impl.global_contact_manager = &require<GlobalContactManager>();
    m_impl.half_plane             = &require<HalfPlane>();

    auto& global_ccd_filter = require<GlobalCCDFilter>();
    global_ccd_filter.add_filter(this);
}

Float HalfPlaneCCDFilter::FilterInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}

muda::CBufferView<Vector3> HalfPlaneCCDFilter::FilterInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> HalfPlaneCCDFilter::FilterInfo::displacements() const noexcept
{
    return m_impl->global_vertex_manager->displacements();
}

muda::CBufferView<IndexT> HalfPlaneCCDFilter::FilterInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}

muda::CBufferView<Vector3> HalfPlaneCCDFilter::FilterInfo::plane_normals() const noexcept
{
    return m_impl->half_plane->normals();
}

muda::CBufferView<Vector3> HalfPlaneCCDFilter::FilterInfo::plane_positions() const noexcept
{
    return m_impl->half_plane->positions();
}

void HalfPlaneCCDFilter::filter_toi(GlobalCCDFilter::FilterInfo& info)
{
    FilterInfo this_info{&m_impl};
    this_info.m_alpha = info.alpha();
    this_info.m_toi   = info.toi();

    do_filter_toi(this_info);
}
}  // namespace uipc::backend::cuda
