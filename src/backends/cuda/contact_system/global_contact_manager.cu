#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_trajectory_filter.h>
#include <sim_engine.h>
#include <contact_system/contact_reporter.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>
#include <uipc/common/unit.h>
#include <uipc/common/zip.h>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalContactManager>
{
  public:
    static U<cuda::GlobalContactManager> create(cuda::SimEngine& engine)
    {
        auto contact_enable_attr =
            engine.world().scene().config().find<IndexT>("contact/enable");
        bool contact_enable = contact_enable_attr->view()[0] != 0;

        auto& types = engine.world().scene().constitution_tabular().types();
        bool  has_inter_primitive_constitution =
            types.find(std::string{builtin::InterPrimitive}) != types.end();

        if(contact_enable || has_inter_primitive_constitution)
            return make_unique<cuda::GlobalContactManager>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalContactManager);

void GlobalContactManager::do_build()
{
    const auto& config = world().scene().config();

    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.global_trajectory_filter = find<GlobalTrajectoryFilter>();


    auto d_hat_attr = config.find<Float>("contact/d_hat");
    m_impl.d_hat    = d_hat_attr->view()[0];

    auto dt_attr = config.find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];

    auto eps_velocity_attr = config.find<Float>("contact/eps_velocity");
    m_impl.eps_velocity    = eps_velocity_attr->view()[0];

    auto cfl_enable_attr = config.find<IndexT>("cfl/enable");
    m_impl.cfl_enabled   = cfl_enable_attr->view()[0] != 0;

    m_impl.kappa = world().scene().contact_tabular().default_model().resistance();
}

muda::CBuffer2DView<IndexT> GlobalContactManager::contact_mask_tabular() const noexcept
{
    return m_impl.contact_mask_tabular;
}

muda::CBuffer2DView<IndexT> GlobalContactManager::subscene_contact_mask_tabular() const noexcept
{
    return m_impl.contact_mask_tabular_subscene;
}

void GlobalContactManager::Impl::init(WorldVisitor& world)
{
    // 1) init tabular
    auto contact_models  = world.scene().contact_tabular().contact_models();
    auto subscene_models = world.scene().subscene_tabular().subscene_models();

    auto attr_topo          = contact_models.find<Vector2i>("topo");
    auto attr_resistance    = contact_models.find<Float>("resistance");
    auto attr_friction_rate = contact_models.find<Float>("friction_rate");
    auto attr_enabled       = contact_models.find<IndexT>("is_enabled");

    auto attr_subscene_topo    = subscene_models.find<Vector2i>("topo");
    auto attr_subscene_enabled = subscene_models.find<IndexT>("is_enabled");

    UIPC_ASSERT(attr_topo != nullptr, "topo is not found in contact tabular");
    UIPC_ASSERT(attr_resistance != nullptr, "resistance is not found in contact tabular");
    UIPC_ASSERT(attr_friction_rate != nullptr, "friction_rate is not found in contact tabular");
    UIPC_ASSERT(attr_enabled != nullptr, "is_enabled is not found in contact tabular");

    UIPC_ASSERT(attr_subscene_topo != nullptr, "subscene topo is not found in contact tabular");
    UIPC_ASSERT(attr_subscene_enabled != nullptr,
                "subscene is_enabled is not found in contact tabular");

    auto topo_view            = attr_topo->view();
    auto resistance_view      = attr_resistance->view();
    auto friction_rate_view   = attr_friction_rate->view();
    auto enabled_view         = attr_enabled->view();
    auto subscene_topo_view   = attr_subscene_topo->view();
    auto subscene_enable_view = attr_subscene_enabled->view();

    auto N  = world.scene().contact_tabular().element_count();
    auto SN = world.scene().contact_tabular().element_count();

    h_contact_tabular.resize(
        N * N, ContactCoeff{.kappa = resistance_view[0], .mu = friction_rate_view[0]});

    h_contact_mask_tabular.resize(N * N, 1);
    h_contact_mask_tabular_subscene.resize(SN * SN, 1);

    for(auto&& [ids, kappa, mu, is_enabled] :
        zip(topo_view, resistance_view, friction_rate_view, enabled_view))
    {

        ContactCoeff coeff{.kappa = kappa, .mu = mu};

        auto upper                    = ids.x() * N + ids.y();
        h_contact_tabular[upper]      = coeff;
        h_contact_mask_tabular[upper] = is_enabled;

        auto lower                    = ids.y() * N + ids.x();
        h_contact_tabular[lower]      = coeff;
        h_contact_mask_tabular[lower] = is_enabled;
    }

    for(auto&& [ids, is_enabled] : zip(subscene_topo_view, subscene_enable_view))
    {
        auto upper                             = ids.x() * SN + ids.y();
        h_contact_mask_tabular_subscene[upper] = is_enabled;

        auto lower                             = ids.y() * SN + ids.x();
        h_contact_mask_tabular_subscene[lower] = is_enabled;
    }

    contact_tabular.resize(muda::Extent2D{N, N});
    contact_tabular.view().copy_from(h_contact_tabular.data());

    contact_mask_tabular.resize(muda::Extent2D{N, N});
    contact_mask_tabular.view().copy_from(h_contact_mask_tabular.data());

    contact_mask_tabular_subscene.resize(muda::Extent2D{SN, SN});
    contact_mask_tabular_subscene.view().copy_from(
        h_contact_mask_tabular_subscene.data());

    // 2) vertex contact info
    vert_is_active_contact.resize(global_vertex_manager->positions().size(), 0);
    vert_disp_norms.resize(global_vertex_manager->positions().size(), 0.0);

    // 3) reporters
    auto contact_reporter_view = contact_reporters.view();
    for(auto&& [i, R] : enumerate(contact_reporter_view))
        R->init();
    for(auto&& [i, R] : enumerate(contact_reporter_view))
        R->m_index = i;
}

void GlobalContactManager::Impl::compute_d_hat()
{
    // TODO: Now do nothing
}

void GlobalContactManager::Impl::compute_adaptive_kappa()
{
    // TODO: Now do nothing
}

Float GlobalContactManager::Impl::compute_cfl_condition()
{
    if(!cfl_enabled)  // if cfl is disabled, just return 1.0
        return 1.0;

    vert_is_active_contact.fill(0);  // clear the active flag

    if(global_trajectory_filter)
    {
        global_trajectory_filter->label_active_vertices();

        auto displacements = global_vertex_manager->displacements();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(displacements.size(),
                   [disps      = displacements.cviewer().name("disp"),
                    disp_norms = vert_disp_norms.viewer().name("disp_norm"),
                    is_contact_active = vert_is_active_contact.viewer().name(
                        "vert_is_contact_active")] __device__(int i) mutable
                   {
                       // if the contact is not active, then the displacement is ignored
                       disp_norms(i) = is_contact_active(i) ? disps(i).norm() : 0.0;
                   });

        DeviceReduce().Max(vert_disp_norms.data(),
                           max_disp_norm.data(),
                           vert_disp_norms.size());

        Float h_max_disp_norm = max_disp_norm;
        return h_max_disp_norm == 0.0 ? 1.0 : std::min(0.5 * d_hat / h_max_disp_norm, 1.0);
    }
    else
    {
        return 1.0;
    }
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void GlobalContactManager::compute_d_hat()
{
    m_impl.compute_d_hat();
}

void GlobalContactManager::compute_adaptive_kappa()
{
    m_impl.compute_adaptive_kappa();
}

Float GlobalContactManager::compute_cfl_condition()
{
    return m_impl.compute_cfl_condition();
}

void GlobalContactManager::init()
{
    m_impl.init(world());
}

Float GlobalContactManager::d_hat() const
{
    return m_impl.d_hat;
}
Float GlobalContactManager::eps_velocity() const
{
    return m_impl.eps_velocity;
}
bool GlobalContactManager::cfl_enabled() const
{
    return m_impl.cfl_enabled;
}

void GlobalContactManager::add_reporter(ContactReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    m_impl.contact_reporters.register_subsystem(*reporter);
}

muda::CBuffer2DView<ContactCoeff> GlobalContactManager::contact_tabular() const noexcept
{
    return m_impl.contact_tabular;
}
}  // namespace uipc::backend::cuda