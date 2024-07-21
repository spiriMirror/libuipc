#include <collision_detection/half_plane_dcd_filter.h>

namespace uipc::backend::cuda
{
void HalfPlaneDCDFilter::detect()
{
    FilterInfo info{&m_impl};
    do_detect(info);
}

void HalfPlaneDCDFilter::do_build()
{
    m_impl.global_vertex_manager = &require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = &require<GlobalSimpicialSurfaceManager>();
    m_impl.global_contact_manager = &require<GlobalContactManager>();
    m_impl.half_plane             = &require<HalfPlane>();
    auto global_dcd_filter        = &require<GlobalDCDFilter>();

    BuildInfo info;
    do_build(info);

    global_dcd_filter->add_filter(this);
}

muda::CBufferView<Vector2i> HalfPlaneDCDFilter::PHs() noexcept
{
    return m_impl.PHs;
}

Float HalfPlaneDCDFilter::FilterInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}

muda::CBufferView<Vector3> HalfPlaneDCDFilter::FilterInfo::plane_normals() const noexcept
{
    return m_impl->half_plane->normals();
}

muda::CBufferView<Vector3> HalfPlaneDCDFilter::FilterInfo::plane_positions() const noexcept
{
    return m_impl->half_plane->positions();
}

muda::CBufferView<Vector3> HalfPlaneDCDFilter::FilterInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<IndexT> HalfPlaneDCDFilter::FilterInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}

void HalfPlaneDCDFilter::FilterInfo::PHs(muda::CBufferView<Vector2i> PHs) noexcept
{
    m_impl->PHs = PHs;
}
}  // namespace uipc::backend::cuda
