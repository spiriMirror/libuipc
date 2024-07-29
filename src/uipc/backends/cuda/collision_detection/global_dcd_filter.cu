#include <collision_detection/global_dcd_filter.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <collision_detection/simplex_dcd_filter.h>
#include <collision_detection/half_plane_dcd_filter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalDCDFilter);

void GlobalDCDFilter::do_build()
{
    if(!world().scene().info()["contact"]["enable"])
    {
        throw SimSystemException("GlobalDCDFilter requires contact to be enabled");
    }

    m_impl.friction_enabled = world().scene().info()["contact"]["friction"]["enable"];

    on_init_scene(
        [this]
        {
            m_impl.simplex_dcd_filter.init();
            m_impl.half_plane_dcd_filter.init();
            m_impl.detect_actions.init();
        });
}

void GlobalDCDFilter::add_filter(SimplexDCDFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input half_plane_dcd_filter is nullptr.");
    m_impl.simplex_dcd_filter.register_subsystem(*filter);
}

void GlobalDCDFilter::add_filter(HalfPlaneDCDFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input half_plane_dcd_filter is nullptr.");
    m_impl.half_plane_dcd_filter.register_subsystem(*filter);
}

void GlobalDCDFilter::add_filter(SimSystem* system, std::function<void()>&& detect_action)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(system != nullptr, "Input system is nullptr.");
    m_impl.detect_actions.register_action(*system, std::move(detect_action));
}

SimplexDCDFilter* GlobalDCDFilter::simplex_filter() const
{
    return m_impl.simplex_dcd_filter.view();
}

HalfPlaneDCDFilter* GlobalDCDFilter::half_plane_filter() const
{
    return m_impl.half_plane_dcd_filter.view();
}

void GlobalDCDFilter::detect()
{
    if(m_impl.simplex_dcd_filter)
        m_impl.simplex_dcd_filter->detect();

    if(m_impl.half_plane_dcd_filter)
        m_impl.half_plane_dcd_filter->detect();

    for(auto& action : m_impl.detect_actions.view())
    {
        action();
    }
}
void GlobalDCDFilter::record_friction_candidates()
{
    if(!m_impl.friction_enabled)
        return;

    if(m_impl.simplex_dcd_filter)
        m_impl.simplex_dcd_filter->record_friction_candidates();

    // TODO:
    // if(m_impl.half_plane_dcd_filter)
    //    m_impl.half_plane_dcd_filter->record_friction_candidates();
}
}  // namespace uipc::backend::cuda