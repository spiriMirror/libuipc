#include <contact_system/vertex_half_plane_frictional_contact.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <utils/make_spd.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>

namespace uipc::backend::cuda
{
void VertexHalfPlaneFrictionalContact::do_build(ContactReporter::BuildInfo& info)
{
    auto& config      = world().scene().config();
    auto  enable_attr = config.find<IndexT>("contact/friction/enable");
    auto  dt_attr     = config.find<Float>("dt");

    if(!enable_attr->view()[0])
    {
        throw SimSystemException("Frictional contact is disabled");
    }

    m_impl.global_trajectory_filter = require<GlobalTrajectoryFilter>();
    m_impl.global_contact_manager   = require<GlobalContactManager>();
    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.vertex_reporter          = require<HalfPlaneVertexReporter>();
    m_impl.dt                       = dt_attr->view()[0];

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

    if(info.gradient_only())
        return;

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
    m_impl.energies      = this_info.m_energies;

    do_compute_energy(this_info);
}

void VertexHalfPlaneFrictionalContact::do_assemble(GlobalContactManager::GradientHessianInfo& info)
{
    ContactInfo this_info{&m_impl};
    this_info.m_gradient_only = info.gradient_only();

    this_info.m_gradients = info.gradients();
    m_impl.gradients      = this_info.m_gradients;
    this_info.m_hessians  = info.hessians();
    m_impl.hessians       = this_info.m_hessians;

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

muda::CBufferView<IndexT> VertexHalfPlaneFrictionalContact::BaseInfo::subscene_element_ids() const
{
    return m_impl->global_vertex_manager->subscene_element_ids();
}

Float VertexHalfPlaneFrictionalContact::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

muda::CBufferView<Float> VertexHalfPlaneFrictionalContact::BaseInfo::d_hats() const
{
    return m_impl->global_vertex_manager->d_hats();
}

Float VertexHalfPlaneFrictionalContact::BaseInfo::dt() const
{
    return m_impl->dt;
}

Float VertexHalfPlaneFrictionalContact::BaseInfo::eps_velocity() const
{
    return m_impl->global_contact_manager->eps_velocity();
}

IndexT VertexHalfPlaneFrictionalContact::BaseInfo::half_plane_vertex_offset() const
{
    return m_impl->vertex_reporter->vertex_offset();
}

muda::BufferView<Float> VertexHalfPlaneFrictionalContact::EnergyInfo::energies() const noexcept
{
    return m_energies;
}

muda::CBufferView<Vector2i> VertexHalfPlaneFrictionalContact::PHs() const noexcept
{
    return m_impl.veretx_half_plane_trajectory_filter->friction_PHs();
}

muda::CBufferView<Float> VertexHalfPlaneFrictionalContact::energies() const noexcept
{
    return m_impl.energies;
}

muda::CDoubletVectorView<Float, 3> VertexHalfPlaneFrictionalContact::gradients() const noexcept
{
    return m_impl.gradients;
}

muda::CTripletMatrixView<Float, 3> VertexHalfPlaneFrictionalContact::hessians() const noexcept
{
    return m_impl.hessians;
}
}  // namespace uipc::backend::cuda

#include <contact_system/contact_exporter.h>

namespace uipc::backend::cuda
{
class VertexHalfPlaneFrictionalContactExporter : public ContactExporter
{
  public:
    using ContactExporter::ContactExporter;

    SimSystemSlot<VertexHalfPlaneFrictionalContact> vertex_half_plane_Frictional_contact;
    SimSystemSlot<HalfPlaneVertexReporter> half_plane_vertex_reporter;

    void do_build(BuildInfo& info) override
    {
        vertex_half_plane_Frictional_contact =
            require<VertexHalfPlaneFrictionalContact>(QueryOptions{.exact = false});
        half_plane_vertex_reporter = require<HalfPlaneVertexReporter>();
    }

    std::string_view get_prim_type() const noexcept override { return "PH+F"; }

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& energy_geo) override
    {
        auto PHs      = vertex_half_plane_Frictional_contact->PHs();
        auto energies = vertex_half_plane_Frictional_contact->energies();

        UIPC_ASSERT(PHs.size() == energies.size(), "PHs and energies must have the same size.");

        energy_geo.instances().resize(PHs.size());
        auto topo = energy_geo.instances().find<Vector2i>("topo");
        if(!topo)
        {
            topo = energy_geo.instances().create<Vector2i>("topo", Vector2i::Zero());
        }

        auto topo_view = view(*topo);
        PHs.copy_to(topo_view.data());
        auto v_offset = half_plane_vertex_reporter->vertex_offset();

        for(Vector2i& topo : topo_view)
            topo[1] += v_offset;


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
        auto PH_grads = vertex_half_plane_Frictional_contact->gradients();
        vert_grad.instances().resize(PH_grads.doublet_count());
        auto i = vert_grad.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_grad.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        PH_grads.indices().copy_to(i_view.data());

        auto grad = vert_grad.instances().find<Vector3>("grad");
        if(!grad)
        {
            grad = vert_grad.instances().create<Vector3>("grad", Vector3::Zero());
        }
        auto grad_view = view(*grad);
        PH_grads.values().copy_to(grad_view.data());
    }

    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess) override
    {
        auto PH_hess = vertex_half_plane_Frictional_contact->hessians();
        vert_hess.instances().resize(PH_hess.triplet_count());
        auto i = vert_hess.instances().find<IndexT>("i");
        if(!i)
        {
            i = vert_hess.instances().create<IndexT>("i", -1);
        }
        auto i_view = view(*i);
        PH_hess.row_indices().copy_to(i_view.data());

        auto j = vert_hess.instances().find<IndexT>("j");
        if(!j)
        {
            j = vert_hess.instances().create<IndexT>("j", -1);
        }
        auto j_view = view(*j);
        PH_hess.col_indices().copy_to(j_view.data());

        auto hess = vert_hess.instances().find<Matrix3x3>("hess");
        if(!hess)
        {
            hess = vert_hess.instances().create<Matrix3x3>("hess", Matrix3x3::Zero());
        }
        auto hess_view = view(*hess);
        PH_hess.values().copy_to(hess_view.data());
    }
};

REGISTER_SIM_SYSTEM(VertexHalfPlaneFrictionalContactExporter);
}  // namespace uipc::backend::cuda