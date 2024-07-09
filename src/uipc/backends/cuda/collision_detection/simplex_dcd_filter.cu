#include <collision_detection/simplex_dcd_filter.h>

namespace uipc::backend::cuda
{
muda::CBufferView<Vector4i> SimplexDCDFilter::PTs() const
{
    return m_impl.PTs;
}
muda::CBufferView<Vector4i> SimplexDCDFilter::EEs() const
{
    return m_impl.EEs;
}
muda::CBufferView<Vector3i> SimplexDCDFilter::PEs() const
{
    return m_impl.PEs;
}
muda::CBufferView<Vector2i> SimplexDCDFilter::PPs() const
{
    return m_impl.PPs;
}

void SimplexDCDFilter::do_build()
{
    m_impl.global_vertex_manager = &require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = &require<GlobalSimpicialSurfaceManager>();
    m_impl.global_contact_manager = &require<GlobalContactManager>();
    auto global_dcd_filter        = &require<GlobalDCDFilter>();

    global_dcd_filter->add_filter(this);
}

void SimplexDCDFilter::detect()
{
    FilterInfo info{&m_impl};
    do_detect(info);
}
Float SimplexDCDFilter::FilterInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}
muda::CBufferView<Vector3> SimplexDCDFilter::FilterInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}
muda::CBufferView<IndexT> SimplexDCDFilter::FilterInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}
muda::CBufferView<Vector2i> SimplexDCDFilter::FilterInfo::surf_edges() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_edges();
}
muda::CBufferView<Vector3i> SimplexDCDFilter::FilterInfo::surf_triangles() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_triangles();
}

void SimplexDCDFilter::FilterInfo::PTs(muda::CBufferView<Vector4i> PTs) noexcept
{
    m_impl->PTs = PTs;
}
void SimplexDCDFilter::FilterInfo::EEs(muda::CBufferView<Vector4i> EEs) noexcept
{
    m_impl->EEs = EEs;
}
void SimplexDCDFilter::FilterInfo::PEs(muda::CBufferView<Vector3i> PEs) noexcept
{
    m_impl->PEs = PEs;
}
void SimplexDCDFilter::FilterInfo::PPs(muda::CBufferView<Vector2i> PPs) noexcept
{
    m_impl->PPs = PPs;
}
}  // namespace uipc::backend::cuda
