#include <contact_system/global_contact_manager.h>
#include <sim_engine.h>
#include <contact_system/contact_reporter.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>
#include <uipc/common/unit.h>
#include <uipc/common/zip.h>
#include <algorithm>
#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <utils/distance.h>
#include <utils/distance/ccd.h>
#include <utils/codim_thickness.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/adaptive_contact_parameter_reporter.h>
#include <cuda_tool/cub.h>

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
namespace
{
    __global__ void compute_cfl_condition_kernel(cuda_tool::CBufferView<Vector3> disps,
                                                 cuda_tool::BufferView<Float>   disp_norms,
                                                 cuda_tool::CBufferView<IndexT> surf_vertices,
                                                 int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        // Stiff-GIPC CFL design: cap by the max displacement over all surface
        // vertices (not only already-active contact vertices), so a fast vertex
        // diving into contact cannot overshoot 0.5*d_hat in one Newton step.
        disp_norms(i) = disps(surf_vertices(i)).norm();
    }

    __global__ void compute_cfl_condition_all_kernel(cuda_tool::CBufferView<Vector3> disps,
                                                     cuda_tool::BufferView<Float> disp_norms,
                                                     int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        disp_norms(i) = disps(i).norm();
    }

    // Stiff-GIPC-style feasible-step pre-cap kernels: over the *active* contact
    // pairs (already within d_hat at the current state), compute the Newton step
    // fraction at which each pair's gap would drop to (1-slackness) of its
    // current value, using the project's exact CCD routines. Pairs that do not
    // come closer within the step contribute no cap (toi = 1).
    __global__ void compute_feasible_step_PT_kernel(cuda_tool::CBufferView<Vector4i> PTs,
                                                    cuda_tool::CBufferView<Vector3>  Ps,
                                                    cuda_tool::CBufferView<Vector3>  dxs,
                                                    cuda_tool::CBufferView<Float> thicknesses,
                                                    cuda_tool::BufferView<Float> tois,
                                                    Float eta,
                                                    int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector4i PT = PTs(i);
        Float thickness = PT_thickness(thicknesses(PT[0]),
                                       thicknesses(PT[1]),
                                       thicknesses(PT[2]),
                                       thicknesses(PT[3]));
        Float toi = 1.0;
        bool  hit = distance::point_triangle_ccd(Ps(PT[0]),
                                                 Ps(PT[1]),
                                                 Ps(PT[2]),
                                                 Ps(PT[3]),
                                                 dxs(PT[0]),
                                                 dxs(PT[1]),
                                                 dxs(PT[2]),
                                                 dxs(PT[3]),
                                                 eta,
                                                 thickness,
                                                 1000,
                                                 toi);
        tois(i) = hit ? toi : Float(1.0);
    }

    __global__ void compute_feasible_step_EE_kernel(cuda_tool::CBufferView<Vector4i> EEs,
                                                    cuda_tool::CBufferView<Vector3>  Ps,
                                                    cuda_tool::CBufferView<Vector3>  dxs,
                                                    cuda_tool::CBufferView<Float> thicknesses,
                                                    cuda_tool::BufferView<Float> tois,
                                                    Float eta,
                                                    int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector4i EE = EEs(i);
        Float thickness = EE_thickness(thicknesses(EE[0]),
                                       thicknesses(EE[1]),
                                       thicknesses(EE[2]),
                                       thicknesses(EE[3]));
        Float toi = 1.0;
        bool  hit = distance::edge_edge_ccd(Ps(EE[0]),
                                            Ps(EE[1]),
                                            Ps(EE[2]),
                                            Ps(EE[3]),
                                            dxs(EE[0]),
                                            dxs(EE[1]),
                                            dxs(EE[2]),
                                            dxs(EE[3]),
                                            eta,
                                            thickness,
                                            1000,
                                            toi);
        tois(i) = hit ? toi : Float(1.0);
    }

    __global__ void compute_feasible_step_PE_kernel(cuda_tool::CBufferView<Vector3i> PEs,
                                                    cuda_tool::CBufferView<Vector3>  Ps,
                                                    cuda_tool::CBufferView<Vector3>  dxs,
                                                    cuda_tool::CBufferView<Float> thicknesses,
                                                    cuda_tool::BufferView<Float> tois,
                                                    Float eta,
                                                    int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector3i PE = PEs(i);
        Float thickness = PE_thickness(thicknesses(PE[0]),
                                       thicknesses(PE[1]),
                                       thicknesses(PE[2]));
        Float toi = 1.0;
        bool  hit = distance::point_edge_ccd(Ps(PE[0]),
                                             Ps(PE[1]),
                                             Ps(PE[2]),
                                             dxs(PE[0]),
                                             dxs(PE[1]),
                                             dxs(PE[2]),
                                             eta,
                                             thickness,
                                             1000,
                                             toi);
        tois(i) = hit ? toi : Float(1.0);
    }

    __global__ void compute_feasible_step_PP_kernel(cuda_tool::CBufferView<Vector2i> PPs,
                                                    cuda_tool::CBufferView<Vector3>  Ps,
                                                    cuda_tool::CBufferView<Vector3>  dxs,
                                                    cuda_tool::CBufferView<Float> thicknesses,
                                                    cuda_tool::BufferView<Float> tois,
                                                    Float eta,
                                                    int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector2i PP = PPs(i);
        Float thickness = PP_thickness(thicknesses(PP[0]), thicknesses(PP[1]));
        Float toi = 1.0;
        bool  hit = distance::point_point_ccd(Ps(PP[0]),
                                              Ps(PP[1]),
                                              dxs(PP[0]),
                                              dxs(PP[1]),
                                              eta,
                                              thickness,
                                              1000,
                                              toi);
        tois(i) = hit ? toi : Float(1.0);
    }
}  // namespace

REGISTER_SIM_SYSTEM(GlobalContactManager);

void GlobalContactManager::do_build()
{
    const auto& config = world().scene().config();

    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.global_trajectory_filter = find<GlobalTrajectoryFilter>();
    m_impl.global_surface_manager   = find<GlobalSimplicialSurfaceManager>();


    auto d_hat_attr = config.find<Float>("contact/d_hat");
    m_impl.d_hat    = d_hat_attr->view()[0];

    m_impl.dt_attr = config.find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");

    auto eps_velocity_attr = config.find<Float>("contact/eps_velocity");
    m_impl.eps_velocity    = eps_velocity_attr->view()[0];

    auto cfl_enable_attr = config.find<IndexT>("cfl/enable");
    m_impl.cfl_enabled   = cfl_enable_attr->view()[0] != 0;

    m_impl.kappa = world().scene().contact_tabular().default_model().resistance();
}

cuda_tool::CBuffer2DView<IndexT> GlobalContactManager::contact_mask_tabular() const noexcept
{
    return m_impl.contact_mask_tabular;
}

cuda_tool::CBuffer2DView<IndexT> GlobalContactManager::subscene_mask_tabular() const noexcept
{
    return m_impl.subscene_mask_tabular;
}

void GlobalContactManager::Impl::init(WorldVisitor& world)
{
    // 0) scene-relative d_hat override: if contact/d_hat_relative > 0,
    //    the gap is relative_dhat * scene diagonal (Stiff-GIPC convention)
    {
        auto rel_attr = world.scene().config().find<Float>("contact/d_hat_relative");
        Float rel     = rel_attr ? rel_attr->view()[0] : 0.0;
        if(rel > 0.0)
        {
            Float diag = global_vertex_manager->scene_diagonal();
            d_hat      = rel * diag;
            logger::info("Contact d_hat (relative): {} = {} x scene_diagonal({})",
                         d_hat, rel, diag);
        }
    }

    // 0.1) scene-relative friction eps_velocity override:
    //    eps_velocity = relative * scene diagonal (Stiff-GIPC convention:
    //    their per-step slip threshold is sqrt(fDhat)*dt = 1e-2*diag*dt,
    //    and our eps_vh = eps_velocity*dt, so eps_velocity = 1e-2*diag)
    {
        auto rel_attr =
            world.scene().config().find<Float>("contact/eps_velocity_relative");
        Float rel = rel_attr ? rel_attr->view()[0] : 0.0;
        if(rel > 0.0)
        {
            Float diag   = global_vertex_manager->scene_diagonal();
            eps_velocity = rel * diag;
            logger::info("Contact eps_velocity (relative): {} = {} x scene_diagonal({})",
                         eps_velocity, rel, diag);
        }
    }

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

    // Default contact stiffness policy:
    //  - user never called default_model() -> use contact/adaptive/min_kappa
    //  - user set it -> clamp into [min_kappa, max_kappa] and remind the range
    //  - negative kappa (adaptive-kappa opt-in) is never clamped
    auto& scene_cfg = world.scene().config();
    Float min_kappa =
        scene_cfg.find<Float>("contact/adaptive/min_kappa")->view()[0];
    Float max_kappa =
        scene_cfg.find<Float>("contact/adaptive/max_kappa")->view()[0];

    Float default_kappa = resistance_view[0];
    if(!world.scene().contact_tabular().default_model_is_user_set())
    {
        default_kappa = min_kappa;
        logger::info(
            "Contact default kappa not set by user; using contact/adaptive/min_kappa = {}",
            min_kappa);
    }
    else if(default_kappa >= 0.0 && (default_kappa < min_kappa || default_kappa > max_kappa))
    {
        logger::warn("Contact default kappa {} is clamped to [{}] (valid range: [{}, {}])",
                     default_kappa,
                     std::clamp(default_kappa, min_kappa, max_kappa),
                     min_kappa,
                     max_kappa);
        default_kappa = std::clamp(default_kappa, min_kappa, max_kappa);
    }

    h_contact_tabular.resize(
        N * N, ContactCoeff{.kappa = default_kappa, .mu = friction_rate_view[0]});

    // set the defined contact model
    for(SizeT i = 0; i < topo_view.size(); ++i)
    {
        auto  ids = topo_view[i];
        Float kappa = resistance_view[i];
        Float mu = friction_rate_view[i];
        auto& is_enabled = enabled_view[i];

        // entry 0 is the default model: use the policy-resolved default kappa
        Float kk = (i == 0) ? default_kappa : kappa;
        if(kk >= 0.0 && (kk < min_kappa || kk > max_kappa))
        {
            Float clamped = std::clamp(kk, min_kappa, max_kappa);
            if(i != 0)
                logger::warn(
                    "Contact kappa {} for model ({},{}) is clamped to {} "
                    "(valid range: [{}, {}])",
                    kk, ids.x(), ids.y(), clamped, min_kappa, max_kappa);
            kk = clamped;
        }

        ContactCoeff coeff{.kappa = kk, .mu = mu};

        auto upper                 = ids.x() * N + ids.y();
        h_contact_tabular[upper]   = coeff;
        mask_map(ids.x(), ids.y()) = is_enabled;


        auto lower                 = ids.y() * N + ids.x();
        h_contact_tabular[lower]   = coeff;
        mask_map(ids.y(), ids.x()) = is_enabled;
    }

    contact_tabular = std::make_shared<cuda_tool::DeviceBuffer2D<ContactCoeff>>();

    contact_tabular->resize(cuda_tool::Extent2D{N, N});
    contact_tabular->view().copy_from(h_contact_tabular.data());

    contact_mask_tabular.resize(cuda_tool::Extent2D{N, N});
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

    subscene_mask_tabular.resize(cuda_tool::Extent2D{SN, SN});
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

    if(global_trajectory_filter)
    {
        auto displacements = global_vertex_manager->displacements();

        using namespace cuda_tool;
        if(displacements.size() > 0)
        {
            if(global_surface_manager)
            {
                // Stiff-GIPC design: max |dx| over all *surface* vertices
                auto surf_verts = global_surface_manager->surf_vertices();
                int  n          = static_cast<int>(surf_verts.size());
                vert_disp_norms.resize(n);
                if(n > 0)
                    compute_cfl_condition_kernel<<<cuda_tool::best_grid_dim(
                                                       n, compute_cfl_condition_kernel),
                                                   cuda_tool::best_block_dim(
                                                       compute_cfl_condition_kernel),
                                                   0,
                                                   nullptr>>>(displacements.cviewer(),
                                                              vert_disp_norms.viewer(),
                                                              surf_verts,
                                                              n);
            }
            else
            {
                // fallback: max |dx| over all vertices
                int n = static_cast<int>(displacements.size());
                vert_disp_norms.resize(n);
                compute_cfl_condition_all_kernel<<<cuda_tool::best_grid_dim(
                                                       n, compute_cfl_condition_all_kernel),
                                                   cuda_tool::best_block_dim(
                                                       compute_cfl_condition_all_kernel),
                                                   0,
                                                   nullptr>>>(displacements.cviewer(),
                                                              vert_disp_norms.viewer(),
                                                              n);
            }
        }

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

Float GlobalContactManager::Impl::compute_feasible_step()
{
    if(!global_trajectory_filter)
        return 1.0;

    auto simplex = global_trajectory_filter->find<SimplexTrajectoryFilter>();
    if(!simplex)
        return 1.0;

    // Stiff-GIPC slackness: keep >= 20% of every active pair's current gap
    constexpr Float eta = 0.2;

    auto positions     = global_vertex_manager->positions();
    auto displacements = global_vertex_manager->displacements();
    auto thicknesses   = global_vertex_manager->thicknesses();

    auto PTs = simplex->PTs();
    auto EEs = simplex->EEs();
    auto PEs = simplex->PEs();
    auto PPs = simplex->PPs();

    SizeT n_pt = PTs.size(), n_ee = EEs.size(), n_pe = PEs.size(), n_pp = PPs.size();
    SizeT total = n_pt + n_ee + n_pe + n_pp;
    if(total == 0)
        return 1.0;

    feasible_tois.resize(total);
    SizeT off = 0;
    // PT
    if(n_pt > 0)
    {
        auto k = compute_feasible_step_PT_kernel;
        int  n = (int)n_pt;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            PTs, positions, displacements, thicknesses, feasible_tois.view(off, n_pt), eta, n);
    }
    off += n_pt;
    // EE
    if(n_ee > 0)
    {
        auto k = compute_feasible_step_EE_kernel;
        int  n = (int)n_ee;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            EEs, positions, displacements, thicknesses, feasible_tois.view(off, n_ee), eta, n);
    }
    off += n_ee;
    // PE
    if(n_pe > 0)
    {
        auto k = compute_feasible_step_PE_kernel;
        int  n = (int)n_pe;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            PEs, positions, displacements, thicknesses, feasible_tois.view(off, n_pe), eta, n);
    }
    off += n_pe;
    // PP
    if(n_pp > 0)
    {
        auto k = compute_feasible_step_PP_kernel;
        int  n = (int)n_pp;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            PPs, positions, displacements, thicknesses, feasible_tois.view(off, n_pp), eta, n);
    }

    cuda_tool::DeviceReduce().Min(feasible_tois.data(), min_feasible_toi.data(), total);
    Float h_min = min_feasible_toi;
    return h_min < 1.0 ? h_min : 1.0;
}

}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
cuda_tool::Buffer2DView<ContactCoeff> GlobalContactManager::AdaptiveParameterInfo::contact_tabular() const noexcept
{
    return m_impl->contact_tabular->view();
}

S<cuda_tool::DeviceBuffer2D<ContactCoeff>> GlobalContactManager::AdaptiveParameterInfo::exchange_contact_tabular(
    S<cuda_tool::DeviceBuffer2D<ContactCoeff>> new_buffer) const noexcept
{
    return std::exchange(m_impl->contact_tabular, new_buffer);
}

Float GlobalContactManager::compute_cfl_condition()
{
    return m_impl.compute_cfl_condition();
}

Float GlobalContactManager::compute_feasible_step()
{
    return m_impl.compute_feasible_step();
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

cuda_tool::CBuffer2DView<ContactCoeff> GlobalContactManager::contact_tabular() const noexcept
{
    return m_impl.contact_tabular->view();
}

}  // namespace uipc::backend::cuda
