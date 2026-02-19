#include <contact_system/simplex_normal_contact.h>
#include <muda/ext/eigen/evd.h>
#include <muda/cub/device/device_merge_sort.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>

namespace uipc::backend::cuda
{
void SimplexNormalContact::do_build(ContactReporter::BuildInfo& info)
{
    m_impl.global_trajectory_filter = require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = require<GlobalContactManager>();
    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    auto dt_attr = world().scene().config().find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];

    BuildInfo this_info;
    do_build(this_info);

    on_init_scene(
        [this]
        {
            m_impl.simplex_trajectory_filter =
                m_impl.global_trajectory_filter->find<SimplexTrajectoryFilter>();
        });
}

void SimplexNormalContact::do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;

    m_impl.PT_count = filter->PTs().size();
    m_impl.EE_count = filter->EEs().size();
    m_impl.PE_count = filter->PEs().size();
    m_impl.PP_count = filter->PPs().size();

    info.energy_count(m_impl.PT_count + m_impl.EE_count + m_impl.PE_count
                      + m_impl.PP_count);
}

void SimplexNormalContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    EnergyInfo this_info{&m_impl};

    auto energies = info.energies();

    SizeT offset = 0;

    this_info.m_PT_energies = energies.subview(offset, m_impl.PT_count);
    m_impl.PT_energies      = this_info.m_PT_energies;
    offset += m_impl.PT_count;

    this_info.m_EE_energies = energies.subview(offset, m_impl.EE_count);
    m_impl.EE_energies      = this_info.m_EE_energies;
    offset += m_impl.EE_count;

    this_info.m_PE_energies = energies.subview(offset, m_impl.PE_count);
    m_impl.PE_energies      = this_info.m_PE_energies;
    offset += m_impl.PE_count;

    this_info.m_PP_energies = energies.subview(offset, m_impl.PP_count);
    m_impl.PP_energies      = this_info.m_PP_energies;
    offset += m_impl.PP_count;

    UIPC_ASSERT(offset == energies.size(),
                "size mismatch, expected {}, got {}",
                energies.size(),
                offset);

    do_compute_energy(this_info);
}

void SimplexNormalContact::do_report_gradient_hessian_extent(GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;
    bool  gradient_only = info.gradient_only();

    m_impl.PT_count = filter->PTs().size();
    m_impl.EE_count = filter->EEs().size();
    m_impl.PE_count = filter->PEs().size();
    m_impl.PP_count = filter->PPs().size();

    auto pt_count = m_impl.PT_count;
    auto ee_count = m_impl.EE_count;
    auto pe_count = m_impl.PE_count;
    auto pp_count = m_impl.PP_count;

    SizeT contact_gradient_count =
        pt_count * 4 + ee_count * 4 + pe_count * 3 + pp_count * 2;
    SizeT contact_hessian_count =
        pt_count * PTHalfHessianSize + ee_count * EEHalfHessianSize
        + pe_count * PEHalfHessianSize + pp_count * PPHalfHessianSize;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(gradient_only ? 0 : contact_hessian_count);
}

void SimplexNormalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    ContactInfo this_info{&m_impl};
    this_info.m_gradient_only = info.gradient_only();
    // gradient
    {
        IndexT offset = 0;
        this_info.m_PT_gradients = info.gradients().subview(offset, m_impl.PT_count * 4);
        m_impl.PT_gradients = this_info.m_PT_gradients;
        offset += m_impl.PT_count * 4;

        this_info.m_EE_gradients = info.gradients().subview(offset, m_impl.EE_count * 4);
        m_impl.EE_gradients = this_info.m_EE_gradients;
        offset += m_impl.EE_count * 4;

        this_info.m_PE_gradients = info.gradients().subview(offset, m_impl.PE_count * 3);
        m_impl.PE_gradients = this_info.m_PE_gradients;
        offset += m_impl.PE_count * 3;

        this_info.m_PP_gradients = info.gradients().subview(offset, m_impl.PP_count * 2);
        m_impl.PP_gradients = this_info.m_PP_gradients;
        offset += m_impl.PP_count * 2;

        UIPC_ASSERT(offset == info.gradients().doublet_count(), "size mismatch");
    }

    // hessian
    {
        if(info.gradient_only())
        {
            this_info.m_PT_hessians = {};
            this_info.m_EE_hessians = {};
            this_info.m_PE_hessians = {};
            this_info.m_PP_hessians = {};

            m_impl.PT_hessians = {};
            m_impl.EE_hessians = {};
            m_impl.PE_hessians = {};
            m_impl.PP_hessians = {};
        }
        else
        {
            IndexT offset = 0;
            this_info.m_PT_hessians =
                info.hessians().subview(offset, m_impl.PT_count * PTHalfHessianSize);
            m_impl.PT_hessians = this_info.m_PT_hessians;
            offset += m_impl.PT_count * PTHalfHessianSize;

            this_info.m_EE_hessians =
                info.hessians().subview(offset, m_impl.EE_count * EEHalfHessianSize);
            m_impl.EE_hessians = this_info.m_EE_hessians;
            offset += m_impl.EE_count * EEHalfHessianSize;

            this_info.m_PE_hessians =
                info.hessians().subview(offset, m_impl.PE_count * PEHalfHessianSize);
            m_impl.PE_hessians = this_info.m_PE_hessians;
            offset += m_impl.PE_count * PEHalfHessianSize;

            this_info.m_PP_hessians =
                info.hessians().subview(offset, m_impl.PP_count * PPHalfHessianSize);
            m_impl.PP_hessians = this_info.m_PP_hessians;
            offset += m_impl.PP_count * PPHalfHessianSize;

            UIPC_ASSERT(offset == info.hessians().triplet_count(), "size mismatch");
        }
    }

    // let subclass to fill in the data
    do_assemble(this_info);
}

muda::CBuffer2DView<ContactCoeff> SimplexNormalContact::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}


muda::CBufferView<Vector4i> SimplexNormalContact::PTs() const
{
    return m_impl.simplex_trajectory_filter->PTs();
}

muda::CBufferView<Float> SimplexNormalContact::PT_energies() const
{
    return m_impl.PT_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexNormalContact::PT_gradients() const
{
    return m_impl.PT_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexNormalContact::PT_hessians() const
{
    return m_impl.PT_hessians;
}

muda::CBufferView<Vector4i> SimplexNormalContact::EEs() const
{
    return m_impl.simplex_trajectory_filter->EEs();
}

muda::CBufferView<Float> SimplexNormalContact::EE_energies() const
{
    return m_impl.EE_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexNormalContact::EE_gradients() const
{
    return m_impl.EE_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexNormalContact::EE_hessians() const
{
    return m_impl.EE_hessians;
}

muda::CBufferView<Vector3i> SimplexNormalContact::PEs() const
{
    return m_impl.simplex_trajectory_filter->PEs();
}

muda::CBufferView<Float> SimplexNormalContact::PE_energies() const
{
    return m_impl.PE_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexNormalContact::PE_gradients() const
{
    return m_impl.PE_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexNormalContact::PE_hessians() const
{
    return m_impl.PE_hessians;
}

muda::CBufferView<Vector2i> SimplexNormalContact::PPs() const
{
    return m_impl.simplex_trajectory_filter->PPs();
}

muda::CBufferView<Float> SimplexNormalContact::PP_energies() const
{
    return m_impl.PP_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexNormalContact::PP_gradients() const
{
    return m_impl.PP_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexNormalContact::PP_hessians() const
{
    return m_impl.PP_hessians;
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

muda::CBufferView<Float> SimplexNormalContact::BaseInfo::d_hats() const
{
    return m_impl->global_vertex_manager->d_hats();
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

#include <contact_system/contact_exporter.h>

namespace uipc::backend::cuda
{
//PT
class SimplexNormalContactPTExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexNormalContact> simplex_normal_contact;

    std::string_view get_prim_type() const noexcept override { return "PT+N"; }

    void do_build(BuildInfo& info) override
    {
        simplex_normal_contact =
            require<SimplexNormalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PTs      = simplex_normal_contact->PTs();
        auto energies = simplex_normal_contact->PT_energies();
        UIPC_ASSERT(PTs.size() == energies.size(), "PTs and energies must have the same size.");
        vert_grad.instances().resize(PTs.size());
        auto topo = vert_grad.instances().find<Vector4i>("topo");
        if(!topo)
        {
            topo = vert_grad.instances().create<Vector4i>("topo", Vector4i::Zero());
        }

        auto topo_view = view(*topo);
        PTs.copy_to(topo_view.data());

        auto energy = vert_grad.instances().find<Float>("energy");
        if(!energy)
        {
            energy = vert_grad.instances().create<Float>("energy", 0.0f);
        }

        auto energy_view = view(*energy);
        energies.copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PT_grads = simplex_normal_contact->PT_gradients();
        vert_grad.instances().resize(PT_grads.doublet_count());
        auto i = vert_grad.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_grad.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        PT_grads.indices().copy_to(i_view.data());

        auto grad = vert_grad.instances().find<Vector3>("grad");
        if(!grad)
        {
            grad = vert_grad.instances().create<Vector3>("grad", Vector3::Zero());
        }
        auto grad_view = view(*grad);
        PT_grads.values().copy_to(grad_view.data());
    }

    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override
    {
        auto PT_hess = simplex_normal_contact->PT_hessians();
        vert_hess.instances().resize(PT_hess.triplet_count());

        auto i = vert_hess.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_hess.instances().create<IndexT>("i", -1);
        }

        auto i_view = view(*i);
        PT_hess.row_indices().copy_to(i_view.data());
        auto j = vert_hess.instances().find<IndexT>("j");
        if(!j)
        {
            j = vert_hess.instances().create<IndexT>("j", -1);
        }

        auto j_view = view(*j);
        PT_hess.col_indices().copy_to(j_view.data());
        auto hess = vert_hess.instances().find<Matrix3x3>("hess");
        if(!hess)
        {
            hess = vert_hess.instances().create<Matrix3x3>("hess", Matrix3x3::Zero());
        }
        auto hess_view = view(*hess);
        PT_hess.values().copy_to(hess_view.data());
    }
};
REGISTER_SIM_SYSTEM(SimplexNormalContactPTExporter);

// EE
class SimplexNormalContactEEExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexNormalContact> simplex_normal_contact;

    std::string_view get_prim_type() const noexcept override { return "EE+N"; }

    void do_build(BuildInfo& info) override
    {
        simplex_normal_contact =
            require<SimplexNormalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& energy_geo) override
    {
        auto EEs      = simplex_normal_contact->EEs();
        auto energies = simplex_normal_contact->EE_energies();
        UIPC_ASSERT(EEs.size() == energies.size(), "EEs and energies must have the same size.");
        energy_geo.instances().resize(EEs.size());
        auto topo = energy_geo.instances().find<Vector4i>("topo");
        if(!topo)
        {
            topo = energy_geo.instances().create<Vector4i>("topo", Vector4i::Zero());
        }

        auto topo_view = view(*topo);
        EEs.copy_to(topo_view.data());

        auto energy = energy_geo.instances().find<Float>("energy");
        if(!energy)
        {
            energy = energy_geo.instances().create<Float>("energy", 0.0f);
        }

        auto energy_view = view(*energy);
        energies.copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto EE_grads = simplex_normal_contact->EE_gradients();
        vert_grad.instances().resize(EE_grads.doublet_count());
        auto i = vert_grad.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_grad.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        EE_grads.indices().copy_to(i_view.data());

        auto grad = vert_grad.instances().find<Vector3>("grad");
        if(!grad)
        {
            grad = vert_grad.instances().create<Vector3>("grad", Vector3::Zero());
        }
        auto grad_view = view(*grad);
        EE_grads.values().copy_to(grad_view.data());
    }

    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override
    {
        auto EE_hess = simplex_normal_contact->EE_hessians();
        vert_hess.instances().resize(EE_hess.triplet_count());
        auto i = vert_hess.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_hess.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        EE_hess.row_indices().copy_to(i_view.data());

        auto j = vert_hess.instances().find<IndexT>("j");
        if(!j)
        {
            j = vert_hess.instances().create<IndexT>("j", -1);
        }
        auto j_view = view(*j);
        EE_hess.col_indices().copy_to(j_view.data());

        auto hess = vert_hess.instances().find<Matrix3x3>("hess");
        if(!hess)
        {
            hess = vert_hess.instances().create<Matrix3x3>("hess", Matrix3x3::Zero());
        }
        auto hess_view = view(*hess);
        EE_hess.values().copy_to(hess_view.data());
    }
};
REGISTER_SIM_SYSTEM(SimplexNormalContactEEExporter);

// PE
class SimplexNormalContactPEExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexNormalContact> simplex_normal_contact;

    std::string_view get_prim_type() const noexcept override { return "PE+N"; }

    void do_build(BuildInfo& info) override
    {
        simplex_normal_contact =
            require<SimplexNormalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PEs      = simplex_normal_contact->PEs();
        auto energies = simplex_normal_contact->PE_energies();
        UIPC_ASSERT(PEs.size() == energies.size(), "PEs and energies must have the same size.");

        vert_grad.instances().resize(PEs.size());
        auto topo = vert_grad.instances().find<Vector3i>("topo");
        if(!topo)
        {
            topo = vert_grad.instances().create<Vector3i>("topo", Vector3i::Zero());
        }

        auto topo_view = view(*topo);
        PEs.copy_to(topo_view.data());

        auto energy = vert_grad.instances().find<Float>("energy");
        if(!energy)
        {
            energy = vert_grad.instances().create<Float>("energy", 0.0f);
        }

        auto energy_view = view(*energy);
        energies.copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PE_grads = simplex_normal_contact->PE_gradients();
        vert_grad.instances().resize(PE_grads.doublet_count());
        auto i = vert_grad.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_grad.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        PE_grads.indices().copy_to(i_view.data());

        auto grad = vert_grad.instances().find<Vector3>("grad");
        if(!grad)
        {
            grad = vert_grad.instances().create<Vector3>("grad", Vector3::Zero());
        }
        auto grad_view = view(*grad);
        PE_grads.values().copy_to(grad_view.data());
    }

    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override
    {
        auto PE_hess = simplex_normal_contact->PE_hessians();
        vert_hess.instances().resize(PE_hess.triplet_count());
        auto i = vert_hess.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_hess.instances().create<IndexT>("i", -1);
        }

        auto i_view = view(*i);
        PE_hess.row_indices().copy_to(i_view.data());

        auto j = vert_hess.instances().find<IndexT>("j");
        if(!j)
        {
            j = vert_hess.instances().create<IndexT>("j", -1);
        }
        auto j_view = view(*j);
        PE_hess.col_indices().copy_to(j_view.data());

        auto hess = vert_hess.instances().find<Matrix3x3>("hess");
        if(!hess)
        {
            hess = vert_hess.instances().create<Matrix3x3>("hess", Matrix3x3::Zero());
        }
        auto hess_view = view(*hess);
        PE_hess.values().copy_to(hess_view.data());
    }
};
REGISTER_SIM_SYSTEM(SimplexNormalContactPEExporter);

// PP
class SimplexNormalContactPPExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexNormalContact> simplex_normal_contact;

    std::string_view get_prim_type() const noexcept override { return "PP+N"; }

    void do_build(BuildInfo& info) override
    {
        simplex_normal_contact =
            require<SimplexNormalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PPs      = simplex_normal_contact->PPs();
        auto energies = simplex_normal_contact->PP_energies();
        UIPC_ASSERT(PPs.size() == energies.size(), "PPs and energies must have the same size.");

        vert_grad.instances().resize(PPs.size());
        auto topo = vert_grad.instances().find<Vector2i>("topo");
        if(!topo)
        {
            topo = vert_grad.instances().create<Vector2i>("topo", Vector2i::Zero());
        }

        auto topo_view = view(*topo);
        PPs.copy_to(topo_view.data());

        auto energy = vert_grad.instances().find<Float>("energy");
        if(!energy)
        {
            energy = vert_grad.instances().create<Float>("energy", 0.0f);
        }

        auto energy_view = view(*energy);
        energies.copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PP_grads = simplex_normal_contact->PP_gradients();
        vert_grad.instances().resize(PP_grads.doublet_count());
        auto i = vert_grad.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_grad.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        PP_grads.indices().copy_to(i_view.data());

        auto grad = vert_grad.instances().find<Vector3>("grad");
        if(!grad)
        {
            grad = vert_grad.instances().create<Vector3>("grad", Vector3::Zero());
        }
        auto grad_view = view(*grad);
        PP_grads.values().copy_to(grad_view.data());
    }

    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override
    {
        auto PP_hess = simplex_normal_contact->PP_hessians();
        vert_hess.instances().resize(PP_hess.triplet_count());
        auto i = vert_hess.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_hess.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        PP_hess.row_indices().copy_to(i_view.data());

        auto j = vert_hess.instances().find<IndexT>("j");
        if(!j)
        {
            j = vert_hess.instances().create<IndexT>("j", -1);
        }
        auto j_view = view(*j);
        PP_hess.col_indices().copy_to(j_view.data());

        auto hess = vert_hess.instances().find<Matrix3x3>("hess");
        if(!hess)
        {
            hess = vert_hess.instances().create<Matrix3x3>("hess", Matrix3x3::Zero());
        }
        auto hess_view = view(*hess);
        PP_hess.values().copy_to(hess_view.data());
    }
};
REGISTER_SIM_SYSTEM(SimplexNormalContactPPExporter);
}  // namespace uipc::backend::cuda
