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
    auto global_dcd_filter        = &require<GlobalDCDFilter>();

    global_dcd_filter->add_filter(this);
}

muda::CBufferView<IndexT> HalfPlaneDCDFilter::Ps() noexcept
{
    return m_impl.Ps;
}

Float HalfPlaneDCDFilter::FilterInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}

muda::CBufferView<Vector3> HalfPlaneDCDFilter::FilterInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<IndexT> HalfPlaneDCDFilter::FilterInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}

void HalfPlaneDCDFilter::FilterInfo::Ps(muda::CBufferView<IndexT> Ps) noexcept
{
    m_impl->Ps = Ps;
}
}  // namespace uipc::backend::cuda
