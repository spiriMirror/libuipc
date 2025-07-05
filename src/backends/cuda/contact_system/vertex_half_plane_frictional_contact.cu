#include <contact_system/vertex_half_plane_frictional_contact.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
void VertexHalfPlaneFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    bool enable = world().scene().info()["contact"]["friction"]["enable"].get<bool>();

    if(!enable)
    {
        throw SimSystemException("Frictional contact is disabled");
    }

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

void VertexHalfPlaneFrictionalContact::do_report_gradient_hessian_extent(
    GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    SizeT count = filter->friction_PHs().size();

    info.gradient_count(count);
    info.hessian_count(count);
}

void VertexHalfPlaneFrictionalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    SizeT count     = filter->friction_PHs().size();
    m_impl.PH_count = count;

    info.energy_count(count);
}

void VertexHalfPlaneFrictionalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;

    EnergyInfo this_info{&m_impl};
    this_info.m_energies = info.energies();

    // let subclass to fill in the data
    do_compute_energy(this_info);
}

void VertexHalfPlaneFrictionalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    ContactInfo this_info{&m_impl};

    this_info.m_gradients = info.gradients();
    this_info.m_hessians  = info.hessians();

    // let subclass to fill in the data
    do_assemble(this_info);
}

muda::CBuffer2DView<ContactCoeff> VertexHalfPlaneFrictionalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector2i> VertexHalfPlaneFrictionalContact::BaseInfo::friction_PHs() const
{
    return m_impl->veretx_half_plane_trajectory_filter->friction_PHs();
}

muda::CBufferView<Vector3> VertexHalfPlaneFrictionalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Float> VertexHalfPlaneFrictionalContact::BaseInfo::thicknesses() const
{
    return m_impl->global_vertex_manager->thicknesses();
}

muda::CBufferView<Vector3> VertexHalfPlaneFrictionalContact::BaseInfo::prev_positions() const
{
    return m_impl->global_vertex_manager->prev_positions();
}

muda::CBufferView<Vector3> VertexHalfPlaneFrictionalContact::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

muda::CBufferView<IndexT> VertexHalfPlaneFrictionalContact::BaseInfo::contact_element_ids() const
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

Float VertexHalfPlaneFrictionalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float VertexHalfPlaneFrictionalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float VertexHalfPlaneFrictionalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}

muda::BufferView<Float> VertexHalfPlaneFrictionalContact::EnergyInfo::energies() const noexcept
{
    return m_energies;
}
}  // namespace uipc::backend::cuda
