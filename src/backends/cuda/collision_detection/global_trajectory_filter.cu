#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/trajectory_filter.h>
#include <contact_system/global_contact_manager.h>
#include <sim_engine.h>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalTrajectoryFilter>
{
  public:
    static U<cuda::GlobalTrajectoryFilter> create(cuda::SimEngine& engine)
    {
        bool contact_enable = engine.world().scene().info()["contact"]["enable"];
        if(contact_enable)
            return make_unique<cuda::GlobalTrajectoryFilter>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend


namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalTrajectoryFilter);

void GlobalTrajectoryFilter::do_build()
{
    m_impl.friction_enabled = world().scene().info()["contact"]["friction"]["enable"];

    m_impl.global_contact_manager = require<GlobalContactManager>();

    on_init_scene([&] { m_impl.init(); });
}

void GlobalTrajectoryFilter::add_filter(TrajectoryFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input TrajectoryFilter is nullptr.");
    m_impl.filters.register_subsystem(*filter);
}

void GlobalTrajectoryFilter::Impl::init()
{
    auto filter_view = filters.view();
    tois.resize(filter_view.size());
    h_tois.resize(filter_view.size());
}

void GlobalTrajectoryFilter::detect(Float alpha)
{
    for(auto filter : m_impl.filters.view())
    {
        DetectInfo info;
        info.m_alpha = alpha;
        filter->detect(info);
    }
}

void GlobalTrajectoryFilter::filter_active()
{
    if(m_impl.global_contact_manager->cfl_enabled())
    {
        auto is_acitive =
            m_impl.global_contact_manager->m_impl.vert_is_active_contact.view();
        is_acitive.fill(0);  // clear the active flag
    }

    for(auto filter : m_impl.filters.view())
    {
        FilterActiveInfo info(&m_impl);
        filter->filter_active(info);
    }
}

Float GlobalTrajectoryFilter::Impl::filter_toi(Float alpha)
{
    auto filter_view = filters.view();
    for(auto&& [i, filter] : enumerate(filter_view))
    {
        FilterTOIInfo info;
        info.m_toi   = muda::VarView<Float>{tois.data() + i};
        info.m_alpha = alpha;
        filter->filter_toi(info);
    }
    tois.view().copy_to(h_tois.data());

    if constexpr(uipc::RUNTIME_CHECK)
    {
        for(auto&& [i, toi] : enumerate(h_tois))
        {
            UIPC_ASSERT(toi > 0.0f, "Invalid toi[{}] value: {}", filter_view[i]->name(), toi);
        }
    }

    auto min_toi = *std::min_element(h_tois.begin(), h_tois.end());

    return min_toi < 1.0 ? min_toi : 1.0;
}

Float GlobalTrajectoryFilter::filter_toi(Float alpha)
{
    return m_impl.filter_toi(alpha);
}

void GlobalTrajectoryFilter::record_friction_candidates()
{
    for(auto filter : m_impl.filters.view())
    {
        RecordFrictionCandidatesInfo info;
        filter->record_friction_candidates(info);
    }
}

void GlobalTrajectoryFilter::label_active_vertices()
{
    for(auto filter : m_impl.filters.view())
    {
        LabelActiveVerticesInfo info(&m_impl);
        filter->label_active_vertices(info);
    }
}

muda::BufferView<IndexT> GlobalTrajectoryFilter::LabelActiveVerticesInfo::vert_is_active() const noexcept
{
    return m_impl->global_contact_manager->m_impl.vert_is_active_contact.view();
}
}  // namespace uipc::backend::cuda
