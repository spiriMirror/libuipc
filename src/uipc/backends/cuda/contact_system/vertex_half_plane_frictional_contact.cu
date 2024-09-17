#include <contact_system/vertex_half_plane_frictional_contact.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <muda/ext/eigen/evd.h>
#include <utils/matrix_assembly_utils.h>

namespace uipc::backend::cuda
{
void VertexHalfPlaneFrictionalContact::do_build()
{
    m_impl.global_trajectory_filter = require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = require<GlobalContactManager>();
    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.veretx_half_plane_trajectory_filter =
        require<VertexHalfPlaneTrajectoryFilter>();
    
    m_impl.dt = world().scene().info()["dt"].get<Float>();

    BuildInfo info;
    do_build(info);

    m_impl.global_contact_manager->add_reporter(this);
}

void VertexHalfPlaneFrictionalContact::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    SizeT count = filter->friction_PHs().size();

    info.gradient_count(count);
    info.hessian_count(count);

    m_impl.loose_resize(m_impl.gradients, count);
    m_impl.loose_resize(m_impl.hessians, count);
}

void VertexHalfPlaneFrictionalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    using namespace muda;

    EnergyInfo this_info{&m_impl};

    auto& filter = m_impl.veretx_half_plane_trajectory_filter;

    auto count = filter->friction_PHs().size();

    m_impl.loose_resize(m_impl.energies, count);
    this_info.m_energies = m_impl.energies.view();

    // let subclass to fill in the data
    do_compute_energy(this_info);

    DeviceReduce().Sum(
        m_impl.energies.data(), info.energy().data(), m_impl.energies.size());

    Float E;
    info.energy().copy_to(&E);

    // spdlog::info("VertexHalfPlaneFrictionalContact energy: {}", E);
}

void VertexHalfPlaneFrictionalContact::Impl::assemble(GlobalContactManager::ContactInfo& info)
{
    using namespace muda;

    auto H3x3 = info.hessian();
    auto G3   = info.gradient();
    auto PHs  = veretx_half_plane_trajectory_filter->friction_PHs();

    // PH
    {
        // Hessians
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(hessians.size(),
                   [PH_H3x3s = hessians.cviewer().name("PH_H3x3"),
                    PHs      = PHs.cviewer().name("PHs"),
                    H3x3s = H3x3.viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix3x3   H3x3 = PH_H3x3s(I);
                       const auto& D2   = PHs(I);
                       auto        P    = D2.x();
                       make_spd<3>(H3x3);
                       H3x3s(I).write(P, P, H3x3);
                   });

        // Gradients
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(gradients.size(),
                   [PH_G3s = gradients.cviewer().name("PHG3"),
                    PHs    = PHs.cviewer().name("PHs"),
                    G3s    = G3.viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& PG3 = PH_G3s(I);
                       const auto& D2  = PHs(I);
                       auto        P   = D2.x();
                       G3s(I).write(P, PG3);
                   });
    }
}

void VertexHalfPlaneFrictionalContact::do_assemble(GlobalContactManager::ContactInfo& info)
{
    ContactInfo this_info{&m_impl};

    this_info.m_gradients = m_impl.gradients;
    this_info.m_hessians  = m_impl.hessians;

    // let subclass to fill in the data
    do_assemble(this_info);

    // assemble the data to the global contact manager
    m_impl.assemble(info);
}

muda::CBuffer2DView<ContactCoeff> VertexHalfPlaneFrictionalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector2i> VertexHalfPlaneFrictionalContact::BaseInfo::PHs() const
{
    return m_impl->veretx_half_plane_trajectory_filter->PHs();
}

muda::CBufferView<Vector3> VertexHalfPlaneFrictionalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
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

muda::BufferView<Vector3> VertexHalfPlaneFrictionalContact::ContactInfo::gradients() const noexcept
{
    return m_gradients;
}
muda::BufferView<Matrix3x3> VertexHalfPlaneFrictionalContact::ContactInfo::hessians() const noexcept
{
    return m_hessians;
}
muda::BufferView<Float> VertexHalfPlaneFrictionalContact::EnergyInfo::energies() const noexcept
{
    return m_energies;
}
}  // namespace uipc::backend::cuda
