#include <collision_detection/global_collision_detector.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<GlobalCollisionDetector>
{
  public:
    static U<GlobalCollisionDetector> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<GlobalCollisionDetector>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(GlobalCollisionDetector);

void GlobalCollisionDetector::do_build()
{
    m_impl.global_contact_manager = find<GlobalContactManager>();
    m_impl.global_vertex_manager  = find<GlobalVertexManager>();
    m_impl.global_surface_manager = find<GlobalSurfaceManager>();
}


}  // namespace uipc::backend::cuda