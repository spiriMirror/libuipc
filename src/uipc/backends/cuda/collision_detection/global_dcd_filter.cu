#include <collision_detection/global_dcd_filter.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <collision_detection/simplex_dcd_filter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalDCDFilter);

void GlobalDCDFilter::add_filter(SimplexDCDFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input simplex_dcd_filter is nullptr.");
    m_impl.simplex_dcd_filter.register_subsystem(*filter);
}

SimplexDCDFilter* GlobalDCDFilter::simplex_filter() const
{
    return m_impl.simplex_dcd_filter.view();
}

void GlobalDCDFilter::do_build()
{
    if(!world().scene().info()["contact"]["enable"])
    {
        throw SimSystemException("GlobalDCDFilter requires contact to be enabled");
    }

    on_init_scene([this] { m_impl.simplex_dcd_filter.init(); });
}

void GlobalDCDFilter::detect()
{
    m_impl.simplex_dcd_filter->detect();
}
}  // namespace uipc::backend::cuda