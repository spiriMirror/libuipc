#include <contact_system/simplex_normal_contact.h>
#include <muda/ext/eigen/evd.h>
#include <muda/cub/device/device_merge_sort.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>

namespace uipc::backend::cuda
{
void SimplexNormalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_trajectory_filter = &require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = &require<GlobalContactManager>();
    m_impl.global_vertex_manager    = &require<GlobalVertexManager>();
    m_impl.dt                       = world().scene().info()["dt"].get<Float>();

    BuildInfo this_info;
    do_build(this_info);

    on_init_scene(
        [this]
        {
            m_impl.simplex_trajectory_filter =
                m_impl.global_trajectory_filter->find<SimplexTrajectoryFilter>();
        });
}

void SimplexNormalContact::Impl::compute_energy(SimplexNormalContact* contact,
                                                GlobalContactManager::EnergyInfo& info)
{
    EnergyInfo this_info{this};

    auto filter = simplex_trajectory_filter;

    PT_count = filter->PTs().size();
    EE_count = filter->EEs().size();
    PE_count = filter->PEs().size();
    PP_count = filter->PPs().size();

    auto count_4 = (PT_count + EE_count);
    auto count_3 = PE_count;
    auto count_2 = PP_count;

    energies.resize(count_4 + count_3 + count_2);

    SizeT offset            = 0;
    this_info.m_PT_energies = energies.view(offset, PT_count);
    offset += PT_count;
    this_info.m_EE_energies = energies.view(offset, EE_count);
    offset += EE_count;
    this_info.m_PE_energies = energies.view(offset, PE_count);
    offset += PE_count;
    this_info.m_PP_energies = energies.view(offset, PP_count);


    contact->do_compute_energy(this_info);
    using namespace muda;

    DeviceReduce().Sum(energies.data(), info.energy().data(), energies.size());
}

void SimplexNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    m_impl.compute_energy(this, info);
}

void SimplexNormalContact::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;

    m_impl.PT_count = filter->PTs().size();
    m_impl.EE_count = filter->EEs().size();
    m_impl.PE_count = filter->PEs().size();
    m_impl.PP_count = filter->PPs().size();


    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;

    // expand to hessian3x3 and graident3
    SizeT contact_gradient_count = 4 * count_4 + 3 * count_3 + 2 * count_2;
    SizeT contact_hessian_count = 4 * 4 * count_4 + 3 * 3 * count_3 + 2 * 2 * count_2;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(contact_hessian_count);
}

void SimplexNormalContact::do_assemble(GlobalContactManager::ContactInfo& info)
{
    ContactInfo this_info{&m_impl};
    // gradient
    {
        IndexT offset = 0;
        this_info.m_PT_gradients = info.gradient().subview(offset, m_impl.PT_count * 4);
        offset += m_impl.PT_count * 4;
        this_info.m_EE_gradients = info.gradient().subview(offset, m_impl.EE_count * 4);
        offset += m_impl.EE_count * 4;
        this_info.m_PE_gradients = info.gradient().subview(offset, m_impl.PE_count * 3);
        offset += m_impl.PE_count * 3;
        this_info.m_PP_gradients = info.gradient().subview(offset, m_impl.PP_count * 2);
        offset += m_impl.PP_count * 2;
        UIPC_ASSERT(offset == info.gradient().doublet_count(), "size mismatch");
    }

    // hessian
    {
        IndexT offset = 0;
        this_info.m_PT_hessians = info.hessian().subview(offset, m_impl.PT_count * 16);
        offset += m_impl.PT_count * 16;
        this_info.m_EE_hessians = info.hessian().subview(offset, m_impl.EE_count * 16);
        offset += m_impl.EE_count * 16;
        this_info.m_PE_hessians = info.hessian().subview(offset, m_impl.PE_count * 9);
        offset += m_impl.PE_count * 9;
        this_info.m_PP_hessians = info.hessian().subview(offset, m_impl.PP_count * 4);
        offset += m_impl.PP_count * 4;
        UIPC_ASSERT(offset == info.hessian().triplet_count(), "size mismatch");
    }

    // let subclass to fill in the data
    do_assemble(this_info);
}

muda::CBuffer2DView<ContactCoeff> SimplexNormalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector4i> SimplexNormalContact::BaseInfo::PTs() const
{
    // return m_impl->PT_constraints.view();
    return m_impl->simplex_trajectory_filter->PTs();
}

muda::CBufferView<Vector4i> SimplexNormalContact::BaseInfo::EEs() const
{
    return m_impl->simplex_trajectory_filter->EEs();
}

muda::CBufferView<Vector3i> SimplexNormalContact::BaseInfo::PEs() const
{
    return m_impl->simplex_trajectory_filter->PEs();
}

muda::CBufferView<Vector2i> SimplexNormalContact::BaseInfo::PPs() const
{
    return m_impl->simplex_trajectory_filter->PPs();
}

muda::CBufferView<Float> SimplexNormalContact::BaseInfo::thicknesses() const
{
    return m_impl->global_vertex_manager->thicknesses();
}

muda::CBufferView<Vector3> SimplexNormalContact::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> SimplexNormalContact::BaseInfo::prev_positions() const
{
    return m_impl->global_vertex_manager->prev_positions();
}

muda::CBufferView<Vector3> SimplexNormalContact::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

muda::CBufferView<IndexT> SimplexNormalContact::BaseInfo::contact_element_ids() const
{
    return m_impl->global_vertex_manager->contact_element_ids();
}

Float SimplexNormalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float SimplexNormalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float SimplexNormalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}
}  // namespace uipc::backend::cuda
