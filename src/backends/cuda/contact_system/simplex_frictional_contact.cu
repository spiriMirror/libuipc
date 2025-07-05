#include <contact_system/simplex_frictional_contact.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
void SimplexFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    bool enable = world().scene().info()["contact"]["friction"]["enable"].get<bool>();

    if(!enable)
    {
        throw SimSystemException("Frictional contact is disabled");
    }

    m_impl.global_trajectory_filter = &require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = &require<GlobalContactManager>();
    m_impl.global_vertex_manager    = &require<GlobalVertexManager>();
    m_impl.dt                       = world().scene().info()["dt"].get<Float>();

    BuildInfo this_info;
    do_build(this_info);

    on_init_scene(
        [this]
        {
            // Ensure that SimplexTrajectoryFilter is already registered in GlobalTrajectoryFilter.
            m_impl.simplex_trajectory_filter =
                m_impl.global_trajectory_filter->find<SimplexTrajectoryFilter>();
        });
}

void SimplexFrictionalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;

    m_impl.PT_count = filter->friction_PTs().size();
    m_impl.EE_count = filter->friction_EEs().size();
    m_impl.PE_count = filter->friction_PEs().size();
    m_impl.PP_count = filter->friction_PPs().size();

    info.energy_count(m_impl.PT_count + m_impl.EE_count + m_impl.PE_count
                      + m_impl.PP_count);
}

void SimplexFrictionalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    EnergyInfo this_info{&m_impl};

    auto energies = info.energies();

    SizeT offset            = 0;
    this_info.m_PT_energies = energies.subview(offset, m_impl.PT_count);
    offset += m_impl.PT_count;
    this_info.m_EE_energies = energies.subview(offset, m_impl.EE_count);
    offset += m_impl.EE_count;
    this_info.m_PE_energies = energies.subview(offset, m_impl.PE_count);
    offset += m_impl.PE_count;
    this_info.m_PP_energies = energies.subview(offset, m_impl.PP_count);

    do_compute_energy(this_info);
}

void SimplexFrictionalContact::do_report_gradient_hessian_extent(GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;

    m_impl.PT_count = filter->friction_PTs().size();
    m_impl.EE_count = filter->friction_EEs().size();
    m_impl.PE_count = filter->friction_PEs().size();
    m_impl.PP_count = filter->friction_PPs().size();

    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;

    // expand to hessian3x3 and graident3
    SizeT contact_gradient_count = 4 * count_4 + 3 * count_3 + 2 * count_2;
    SizeT contact_hessian_count = (4 * 4) * count_4 + (3 * 3) * count_3 + (2 * 2) * count_2;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(contact_hessian_count);
}


void SimplexFrictionalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    ContactInfo this_info{&m_impl};
    // gradient
    {
        IndexT offset = 0;
        this_info.m_PT_gradients = info.gradients().subview(offset, m_impl.PT_count * 4);
        offset += m_impl.PT_count * 4;
        this_info.m_EE_gradients = info.gradients().subview(offset, m_impl.EE_count * 4);
        offset += m_impl.EE_count * 4;
        this_info.m_PE_gradients = info.gradients().subview(offset, m_impl.PE_count * 3);
        offset += m_impl.PE_count * 3;
        this_info.m_PP_gradients = info.gradients().subview(offset, m_impl.PP_count * 2);
        offset += m_impl.PP_count * 2;
        UIPC_ASSERT(offset == info.gradients().doublet_count(), "size mismatch");
    }

    // hessian
    {
        IndexT offset = 0;
        this_info.m_PT_hessians = info.hessians().subview(offset, m_impl.PT_count * 16);
        offset += m_impl.PT_count * 16;
        this_info.m_EE_hessians = info.hessians().subview(offset, m_impl.EE_count * 16);
        offset += m_impl.EE_count * 16;
        this_info.m_PE_hessians = info.hessians().subview(offset, m_impl.PE_count * 9);
        offset += m_impl.PE_count * 9;
        this_info.m_PP_hessians = info.hessians().subview(offset, m_impl.PP_count * 4);
        offset += m_impl.PP_count * 4;
        UIPC_ASSERT(offset == info.hessians().triplet_count(), "size mismatch");
    }

    // let subclass to fill in the data
    do_assemble(this_info);
}

muda::CBuffer2DView<ContactCoeff> SimplexFrictionalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector4i> SimplexFrictionalContact::BaseInfo::friction_PTs() const
{
    return m_impl->simplex_trajectory_filter->friction_PTs();
}

muda::CBufferView<Vector4i> SimplexFrictionalContact::BaseInfo::friction_EEs() const
{
    return m_impl->simplex_trajectory_filter->friction_EEs();
}

muda::CBufferView<Vector3i> SimplexFrictionalContact::BaseInfo::friction_PEs() const
{
    return m_impl->simplex_trajectory_filter->friction_PEs();
}

muda::CBufferView<Vector2i> SimplexFrictionalContact::BaseInfo::friction_PPs() const
{
    return m_impl->simplex_trajectory_filter->friction_PPs();
}

muda::CBufferView<Vector3> SimplexFrictionalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> SimplexFrictionalContact::BaseInfo::prev_positions() const
{
    return m_impl->global_vertex_manager->prev_positions();
}

muda::CBufferView<Vector3> SimplexFrictionalContact::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

muda::CBufferView<Float> SimplexFrictionalContact::BaseInfo::thicknesses() const
{
    return m_impl->global_vertex_manager->thicknesses();
}

muda::CBufferView<IndexT> SimplexFrictionalContact::BaseInfo::contact_element_ids() const
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

Float SimplexFrictionalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float SimplexFrictionalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float SimplexFrictionalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}
}  // namespace uipc::backend::cuda
