#include <collision_detection/global_dcd_filter.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <collision_detection/simplex_dcd_filter.h>
#include <sim_engine.h>
#include <collision_detection/dcd_filter.h>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalDCDFilter>
{
  public:
    static U<cuda::GlobalDCDFilter> create(cuda::SimEngine& engine)
    {
        if(engine.world().scene().info()["contact"]["enable"])
            return make_unique<cuda::GlobalDCDFilter>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalDCDFilter);

void GlobalDCDFilter::add_filter(SimplexDCDFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input filter is nullptr.");
    UIPC_ASSERT(m_impl.filter == nullptr,
                "Filter({}) already set, yours ({}).",
                m_impl.filter->name(),
                filter->name());
    m_impl.filter = filter;
}

SimplexDCDFilter* GlobalDCDFilter::simplex_filter() const
{
    return m_impl.filter;
}

void GlobalDCDFilter::do_build()
{
    if(!world().scene().info()["contact"]["enable"])
    {
        throw SimEngineException("GlobalDCDFilter requires contact to be enabled");
    }
}

void GlobalDCDFilter::detect()
{
    m_impl.filter->detect();
}
}  // namespace uipc::backend::cuda