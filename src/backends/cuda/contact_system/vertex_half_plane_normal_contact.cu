#include <contact_system/vertex_half_plane_normal_contact.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
void VertexHalfPlaneNormalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_trajectory_filter = require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = require<GlobalContactManager>();
    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.dt                       = world().scene().info()["dt"].get<Float>();

    BuildInfo this_info;
    do_build(this_info);

    on_init_scene(
        [this]
        {
            m_impl.veretx_half_plane_trajectory_filter =
                m_impl.global_trajectory_filter->find<VertexHalfPlaneTrajectoryFilter>();
        });
}

void VertexHalfPlaneNormalContact::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    SizeT count = filter->PHs().size();

    info.gradient_count(count);
    info.hessian_count(count);
}

void VertexHalfPlaneNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;

    EnergyInfo this_info{&m_impl};

    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    auto count = filter->PHs().size();

    m_impl.loose_resize(m_impl.energies, count);
    this_info.m_energies = m_impl.energies.view();

    // let subclass to fill in the data
    do_compute_energy(this_info);

    DeviceReduce().Sum(
        m_impl.energies.data(), info.energy().data(), m_impl.energies.size());

    Float E;
    info.energy().copy_to(&E);

    // spdlog::info("VertexHalfPlaneNormalContact energy: {}", E);
}

void VertexHalfPlaneNormalContact::do_assemble(GlobalContactManager::ContactInfo& info)
{
    ContactInfo this_info{&m_impl};

    this_info.m_gradients = info.gradient();
    this_info.m_hessians  = info.hessian();

    // let subclass to fill in the data
    do_assemble(this_info);
}

muda::CBuffer2DView<ContactCoeff> VertexHalfPlaneNormalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector2i> VertexHalfPlaneNormalContact::BaseInfo::PHs() const
{
    return m_impl->veretx_half_plane_trajectory_filter->PHs();
}

muda::CBufferView<Vector3> VertexHalfPlaneNormalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> VertexHalfPlaneNormalContact::BaseInfo::prev_positions() const
{
    return m_impl->global_vertex_manager->prev_positions();
}

muda::CBufferView<Vector3> VertexHalfPlaneNormalContact::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

muda::CBufferView<Float> VertexHalfPlaneNormalContact::BaseInfo::thicknesses() const
{
    return m_impl->global_vertex_manager->thicknesses();
}

muda::CBufferView<IndexT> VertexHalfPlaneNormalContact::BaseInfo::contact_element_ids() const
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

Float VertexHalfPlaneNormalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float VertexHalfPlaneNormalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float VertexHalfPlaneNormalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}

muda::BufferView<Float> VertexHalfPlaneNormalContact::EnergyInfo::energies() const noexcept
{
    return m_energies;
}
}  // namespace uipc::backend::cuda
