#include <contact_system/simplex_frictional_contact.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
void SimplexFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    auto& config      = world().scene().config();
    auto  enable_attr = config.find<IndexT>("contact/friction/enable");

    if(!enable_attr->view()[0])
    {
        throw SimSystemException("Frictional contact is disabled");
    }

    m_impl.global_trajectory_filter = &require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = &require<GlobalContactManager>();
    m_impl.global_vertex_manager    = &require<GlobalVertexManager>();

    auto dt_attr = world().scene().config().find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];

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

void SimplexFrictionalContact::do_report_gradient_hessian_extent(GlobalContactManager::GradientHessianExtentInfo& info)
{
    auto& filter = m_impl.simplex_trajectory_filter;
    bool  gradient_only = info.gradient_only();

    m_impl.PT_count = filter->friction_PTs().size();
    m_impl.EE_count = filter->friction_EEs().size();
    m_impl.PE_count = filter->friction_PEs().size();
    m_impl.PP_count = filter->friction_PPs().size();

    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;

    // expand to hessian3x3 and graident3
    SizeT contact_gradient_count = 4 * count_4 + 3 * count_3 + 2 * count_2;
    SizeT contact_hessian_count = PTHalfHessianSize * m_impl.PT_count
                                  + EEHalfHessianSize * m_impl.EE_count
                                  + PEHalfHessianSize * m_impl.PE_count
                                  + PPHalfHessianSize * m_impl.PP_count;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(gradient_only ? 0 : contact_hessian_count);
}


void SimplexFrictionalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
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

muda::CBufferView<Vector4i> SimplexFrictionalContact::PTs() const
{
    return m_impl.simplex_trajectory_filter->friction_PTs();
}

muda::CBufferView<Float> SimplexFrictionalContact::PT_energies() const
{
    return m_impl.PT_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexFrictionalContact::PT_gradients() const
{
    return m_impl.PT_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexFrictionalContact::PT_hessians() const
{
    return m_impl.PT_hessians;
}

muda::CBufferView<Vector4i> SimplexFrictionalContact::EEs() const
{
    return m_impl.simplex_trajectory_filter->friction_EEs();
}

muda::CBufferView<Float> SimplexFrictionalContact::EE_energies() const
{
    return m_impl.EE_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexFrictionalContact::EE_gradients() const
{
    return m_impl.EE_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexFrictionalContact::EE_hessians() const
{
    return m_impl.EE_hessians;
}

muda::CBufferView<Vector3i> SimplexFrictionalContact::PEs() const
{
    return m_impl.simplex_trajectory_filter->friction_PEs();
}

muda::CBufferView<Float> SimplexFrictionalContact::PE_energies() const
{
    return m_impl.PE_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexFrictionalContact::PE_gradients() const
{
    return m_impl.PE_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexFrictionalContact::PE_hessians() const
{
    return m_impl.PE_hessians;
}

muda::CBufferView<Vector2i> SimplexFrictionalContact::PPs() const
{
    return m_impl.simplex_trajectory_filter->friction_PPs();
}

muda::CBufferView<Float> SimplexFrictionalContact::PP_energies() const
{
    return m_impl.PP_energies;
}

muda::CDoubletVectorView<Float, 3> SimplexFrictionalContact::PP_gradients() const
{
    return m_impl.PP_gradients;
}

muda::CTripletMatrixView<Float, 3> SimplexFrictionalContact::PP_hessians() const
{
    return m_impl.PP_hessians;
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

muda::CBufferView<Float> SimplexFrictionalContact::BaseInfo::d_hats() const
{
    return m_impl->global_vertex_manager->d_hats();
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

#include <contact_system/contact_exporter.h>

namespace uipc::backend::cuda
{
//PT
class SimplexFrictionalContactPTExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexFrictionalContact> simplex_frictional_contact;

    std::string_view get_prim_type() const noexcept override { return "PT+F"; }

    void do_build(BuildInfo& info) override
    {
        simplex_frictional_contact =
            require<SimplexFrictionalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PTs = simplex_frictional_contact->PTs();
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
        simplex_frictional_contact->PT_energies().copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PT_grads = simplex_frictional_contact->PT_gradients();
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
        auto PT_hess = simplex_frictional_contact->PT_hessians();
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
REGISTER_SIM_SYSTEM(SimplexFrictionalContactPTExporter);

// EE
class SimplexFrictionalContactEEExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexFrictionalContact> simplex_frictional_contact;

    std::string_view get_prim_type() const noexcept override { return "EE+F"; }

    void do_build(BuildInfo& info) override
    {
        simplex_frictional_contact =
            require<SimplexFrictionalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto EEs = simplex_frictional_contact->EEs();
        vert_grad.instances().resize(EEs.size());
        auto topo = vert_grad.instances().find<Vector4i>("topo");
        if(!topo)
        {
            topo = vert_grad.instances().create<Vector4i>("topo", Vector4i::Zero());
        }

        auto topo_view = view(*topo);
        EEs.copy_to(topo_view.data());

        auto energy = vert_grad.instances().find<Float>("energy");
        if(!energy)
        {
            energy = vert_grad.instances().create<Float>("energy", 0.0f);
        }

        auto energy_view = view(*energy);
        simplex_frictional_contact->EE_energies().copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto EE_grads = simplex_frictional_contact->EE_gradients();
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
        auto EE_hess = simplex_frictional_contact->EE_hessians();
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
REGISTER_SIM_SYSTEM(SimplexFrictionalContactEEExporter);

// PE
class SimplexFrictionalContactPEExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexFrictionalContact> simplex_frictional_contact;

    std::string_view get_prim_type() const noexcept override { return "PE+F"; }

    void do_build(BuildInfo& info) override
    {
        simplex_frictional_contact =
            require<SimplexFrictionalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PEs = simplex_frictional_contact->PEs();
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
        simplex_frictional_contact->PE_energies().copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PE_grads = simplex_frictional_contact->PE_gradients();
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
        auto PE_hess = simplex_frictional_contact->PE_hessians();
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
REGISTER_SIM_SYSTEM(SimplexFrictionalContactPEExporter);

// PP
class SimplexFrictionalContactPPExporter final : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<SimplexFrictionalContact> simplex_frictional_contact;

    std::string_view get_prim_type() const noexcept override { return "PP+F"; }

    void do_build(BuildInfo& info) override
    {
        simplex_frictional_contact =
            require<SimplexFrictionalContact>(QueryOptions{.exact = false});
    }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PPs = simplex_frictional_contact->PPs();
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
        simplex_frictional_contact->PP_energies().copy_to(energy_view.data());
    }

    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad) override
    {
        auto PP_grads = simplex_frictional_contact->PP_gradients();
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
        auto PP_hess = simplex_frictional_contact->PP_hessians();
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
REGISTER_SIM_SYSTEM(SimplexFrictionalContactPPExporter);
}  // namespace uipc::backend::cuda