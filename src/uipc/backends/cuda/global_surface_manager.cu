#include <global_surface_manager.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalSurfaceManager);

void GlobalSurfaceManager::do_build()
{
    m_impl.global_vertex_manager = find<GlobalVertexManager>();
}
void GlobalSurfaceManager::init_surface_info() {}

void GlobalSurfaceManager::rebuild_surface_info() {}
}  // namespace uipc::backend::cuda
