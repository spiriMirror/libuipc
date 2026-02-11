#include <contact_system/global_contact_manager.h>
#include <sim_engine.h>
#include <contact_system/contact_reporter.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>
#include <uipc/common/unit.h>
#include <uipc/common/zip.h>
#include <collision_detection/global_trajectory_filter.h>
#include <contact_system/adaptive_contact_parameter_reporter.h>

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
        if(contact_enable)
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

muda::CBuffer2DView<IndexT> GlobalContactManager::subscene_mask_tabular() const noexcept
{
    return m_impl.subscene_mask_tabular;
}

void GlobalContactManager::Impl::init(WorldVisitor& world)
{
    // 1) init tabular
    _build_contact_tabular(world);
    _build_subscene_tabular(world);


    // 2) vertex contact info
    vert_is_active_contact.resize(global_vertex_manager->positions().size(), 0);
    vert_disp_norms.resize(global_vertex_manager->positions().size(), 0.0);

    // 3) reporters
    auto contact_reporter_view = contact_reporters.view();
    for(auto&& [i, R] : enumerate(contact_reporter_view))
        R->init();
    for(auto&& [i, R] : enumerate(contact_reporter_view))
        R->m_index = i;
    if(adaptive_contact_parameter_reporter)
    {
        adaptive_contact_parameter_reporter->init();
    }
}

using MaskMatrix = Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


void GlobalContactManager::Impl::_build_contact_tabular(WorldVisitor& world)
{
    // Specification:
    // https://spirimirror.github.io/libuipc-doc/specification/#contact-tabular

    auto contact_models = world.scene().contact_tabular().contact_models();

    auto attr_topo          = contact_models.find<Vector2i>("topo");
    auto attr_resistance    = contact_models.find<Float>("resistance");
    auto attr_friction_rate = contact_models.find<Float>("friction_rate");
    auto attr_enabled       = contact_models.find<IndexT>("is_enabled");

    UIPC_ASSERT(attr_topo != nullptr, "topo is not found in contact tabular");
    UIPC_ASSERT(attr_resistance != nullptr, "resistance is not found in contact tabular");
    UIPC_ASSERT(attr_friction_rate != nullptr, "friction_rate is not found in contact tabular");
    UIPC_ASSERT(attr_enabled != nullptr, "is_enabled is not found in contact tabular");

    auto topo_view          = attr_topo->view();
    auto resistance_view    = attr_resistance->view();
    auto friction_rate_view = attr_friction_rate->view();
    auto enabled_view       = attr_enabled->view();

    auto N = world.scene().contact_tabular().element_count();

    // if no contact model is defined, then take the default one
    // so it can be on or off, depending on the default_model setting
    h_contact_mask_tabular.resize(N * N, enabled_view[0]);

    auto mask_map = Eigen::Map<MaskMatrix>(h_contact_mask_tabular.data(), N, N);

    h_contact_tabular.resize(
        N * N, ContactCoeff{.kappa = resistance_view[0], .mu = friction_rate_view[0]});

    // set the defined contact model
    for(auto&& [ids, kappa, mu, is_enabled] :
        zip(topo_view, resistance_view, friction_rate_view, enabled_view))
    {
        ContactCoeff coeff{.kappa = kappa, .mu = mu};

        auto upper                 = ids.x() * N + ids.y();
        h_contact_tabular[upper]   = coeff;
        mask_map(ids.x(), ids.y()) = is_enabled;


        auto lower                 = ids.y() * N + ids.x();
        h_contact_tabular[lower]   = coeff;
        mask_map(ids.y(), ids.x()) = is_enabled;
    }

    contact_tabular = std::make_shared<muda::DeviceBuffer2D<ContactCoeff>>();

    contact_tabular->resize(muda::Extent2D{N, N});
    contact_tabular->view().copy_from(h_contact_tabular.data());

    contact_mask_tabular.resize(muda::Extent2D{N, N});
    contact_mask_tabular.view().copy_from(h_contact_mask_tabular.data());
}

void GlobalContactManager::Impl::_build_subscene_tabular(WorldVisitor& world)
{
    // Specification:
    // https://spirimirror.github.io/libuipc-doc/specification/#subscene-tabular

    auto subscene_models = world.scene().subscene_tabular().subscene_models();

    auto topo       = subscene_models.find<Vector2i>("topo");
    auto is_enabled = subscene_models.find<IndexT>("is_enabled");


    UIPC_ASSERT(topo != nullptr, "subscene topo is not found in contact tabular");
    UIPC_ASSERT(is_enabled != nullptr, "subscene is_enabled is not found in contact tabular");

    auto topo_view    = topo->view();
    auto enabled_view = is_enabled->view();
    auto SN           = world.scene().subscene_tabular().element_count();

    h_subcene_mask_tabular.resize(SN * SN);
    auto mask_map = Eigen::Map<MaskMatrix>(h_subcene_mask_tabular.data(), SN, SN);

    // According to Specification:
    // 1. default turn off the contact between two different subscenes
    // 2. enable self-scene-contact
    mask_map.setIdentity();

    for(auto&& [ids, is_enabled] : zip(topo_view, enabled_view))
    {
        mask_map(ids.x(), ids.y()) = is_enabled;
        mask_map(ids.y(), ids.x()) = is_enabled;
    }

    subscene_mask_tabular.resize(muda::Extent2D{SN, SN});
    subscene_mask_tabular.view().copy_from(h_subcene_mask_tabular.data());
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
muda::Buffer2DView<ContactCoeff> GlobalContactManager::AdaptiveParameterInfo::contact_tabular() const noexcept
{
    return m_impl->contact_tabular->view();
}

S<muda::DeviceBuffer2D<ContactCoeff>> GlobalContactManager::AdaptiveParameterInfo::exchange_contact_tabular(
    S<muda::DeviceBuffer2D<ContactCoeff>> new_buffer) const noexcept
{
    return std::exchange(m_impl->contact_tabular, new_buffer);
}

Float GlobalContactManager::compute_cfl_condition()
{
    return m_impl.compute_cfl_condition();
}

void GlobalContactManager::init()
{
    m_impl.init(world());
}

void GlobalContactManager::compute_adaptive_parameters()
{
    if(!m_impl.adaptive_contact_parameter_reporter)
        return;

    auto info = AdaptiveParameterInfo(&m_impl);
    m_impl.adaptive_contact_parameter_reporter->compute_parameters(info);
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
    m_impl.contact_reporters.register_sim_system(*reporter);
}

void GlobalContactManager::add_reporter(AdaptiveContactParameterReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(!m_impl.adaptive_contact_parameter_reporter,
                "AdaptiveContactParameterReporter is already registered, name: {}",
                m_impl.adaptive_contact_parameter_reporter->name());
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    m_impl.adaptive_contact_parameter_reporter.register_sim_system(*reporter);
}

muda::CBuffer2DView<ContactCoeff> GlobalContactManager::contact_tabular() const noexcept
{
    return m_impl.contact_tabular->view();
}

}  // namespace uipc::backend::cuda
