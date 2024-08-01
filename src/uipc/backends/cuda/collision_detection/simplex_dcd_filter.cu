#include <collision_detection/simplex_dcd_filter.h>

namespace uipc::backend::cuda
{
void SimplexDCDFilter::do_build()
{
    m_impl.global_vertex_manager = &require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = &require<GlobalSimpicialSurfaceManager>();
    m_impl.global_contact_manager = &require<GlobalContactManager>();
    auto global_dcd_filter        = &require<GlobalDCDFilter>();

    global_dcd_filter->add_filter(this);
}

void SimplexDCDFilter::detect_trajectory_candidates(Float alpha)
{
    DetectTrajectoryInfo info{&m_impl};
    info.m_alpha = alpha;
    do_detect_trajectory_candidates(info);
}

void SimplexDCDFilter::filter_candidates()
{
    FilterInfo info{&m_impl};
    do_filter_candidates(info);

    spdlog::info("SimplexDCDFilter PTs: {}, EEs: {}, PEs: {}, PPs: {}",
                 m_impl.PTs.size(),
                 m_impl.EEs.size(),
                 m_impl.PEs.size(),
                 m_impl.PPs.size());
}

void SimplexDCDFilter::Impl::record_friction_candidates()
{
    loose_resize(friction_PT, PTs.size());
    friction_PT.view().copy_from(PTs);

    loose_resize(friction_EE, EEs.size());
    friction_EE.view().copy_from(EEs);

    loose_resize(friction_PE, PEs.size());
    friction_PE.view().copy_from(PEs);

    loose_resize(friction_PP, PPs.size());
    friction_PP.view().copy_from(PPs);

    spdlog::info("SimplexDCDFilter Friction PT: {}, EE: {}, PE: {}, PP: {}",
                 friction_PT.size(),
                 friction_EE.size(),
                 friction_PE.size(),
                 friction_PP.size());
}

void SimplexDCDFilter::record_friction_candidates()
{
    m_impl.record_friction_candidates();
}

Float SimplexDCDFilter::BaseInfo::d_hat() const noexcept
{
    return m_impl->global_contact_manager->d_hat();
}

muda::CBufferView<Vector3> SimplexDCDFilter::BaseInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<IndexT> SimplexDCDFilter::BaseInfo::surf_vertices() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_vertices();
}

muda::CBufferView<Vector2i> SimplexDCDFilter::BaseInfo::surf_edges() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_edges();
}

muda::CBufferView<Vector3i> SimplexDCDFilter::BaseInfo::surf_triangles() const noexcept
{
    return m_impl->global_simplicial_surface_manager->surf_triangles();
}

//void SimplexDCDFilter::DetectInfo::PTs(muda::CBufferView<Vector4i> PTs) noexcept
//{
//    m_impl->PTs = PTs;
//}
//void SimplexDCDFilter::DetectInfo::EEs(muda::CBufferView<Vector4i> EEs) noexcept
//{
//    m_impl->EEs = EEs;
//}
//void SimplexDCDFilter::DetectInfo::PEs(muda::CBufferView<Vector3i> PEs) noexcept
//{
//    m_impl->PEs = PEs;
//}
//void SimplexDCDFilter::DetectInfo::PPs(muda::CBufferView<Vector2i> PPs) noexcept
//{
//    m_impl->PPs = PPs;
//}

muda::CBufferView<Vector4i> SimplexDCDFilter::PTs() const noexcept
{
    return m_impl.PTs;
}
muda::CBufferView<Vector4i> SimplexDCDFilter::EEs() const noexcept
{
    return m_impl.EEs;
}
muda::CBufferView<Vector3i> SimplexDCDFilter::PEs() const noexcept
{
    return m_impl.PEs;
}
muda::CBufferView<Vector2i> SimplexDCDFilter::PPs() const noexcept
{
    return m_impl.PPs;
}

muda::CBufferView<Vector4i> SimplexDCDFilter::friction_PTs() const noexcept
{
    return m_impl.friction_PT;
}

muda::CBufferView<Vector4i> SimplexDCDFilter::friction_EEs() const noexcept
{
    return m_impl.friction_EE;
}

muda::CBufferView<Vector3i> SimplexDCDFilter::friction_PEs() const noexcept
{
    return m_impl.friction_PE;
}

muda::CBufferView<Vector2i> SimplexDCDFilter::friction_PPs() const noexcept
{
    return m_impl.friction_PP;
}
muda::CBufferView<Vector3> SimplexDCDFilter::DetectTrajectoryInfo::displacements() const noexcept
{
    return m_impl->global_vertex_manager->displacements();
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
