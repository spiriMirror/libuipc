#include <collision_detection/simplex_ccd_filter.h>

namespace uipc::backend::cuda
{
void SimplexCCDFilter::do_build()
{
    m_impl.global_vertex_manager = &require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = &require<GlobalSimpicialSurfaceManager>();
    m_impl.global_contact_manager = &require<GlobalContactManager>();

    auto& global_ccd_filter = require<GlobalCCDFilter>();
    global_ccd_filter.add_filter(this);
}

Float SimplexCCDFilter::FilterInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}
muda::CBufferView<Vector3> SimplexCCDFilter::FilterInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}
muda::CBufferView<Vector3> SimplexCCDFilter::FilterInfo::displacements() const noexcept
{
    return m_impl->global_vertex_manager->displacements();
}
muda::CBufferView<IndexT> SimplexCCDFilter::FilterInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}
muda::CBufferView<Vector2i> SimplexCCDFilter::FilterInfo::surf_edges() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_edges();
}
muda::CBufferView<Vector3i> SimplexCCDFilter::FilterInfo::surf_triangles() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_triangles();
}

void SimplexCCDFilter::filter_toi(GlobalCCDFilter::FilterInfo& info)
{
    FilterInfo this_info{&m_impl};
    this_info.m_alpha = info.alpha();
    this_info.m_toi   = info.toi();

    do_filter_toi(this_info);
}
}  // namespace uipc::backend::cuda
