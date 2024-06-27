#include <contact_system/global_contact_manager.h>
#include <sim_engine.h>
namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<GlobalContactManager>
{
  public:
    static U<GlobalContactManager> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<GlobalContactManager>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(GlobalContactManager);

void GlobalContactManager::do_build()
{
    m_impl.global_vertex_manager = find<GlobalVertexManager>();
    const auto& info             = world().scene().info();
    m_impl.related_d_hat         = info["contact"]["d_hat"].get<Float>();
}

void GlobalContactManager::Impl::update_contact_parameters()
{
    AABB vert_aabb = global_vertex_manager->vertex_bounding_box();
    d_hat          = related_d_hat * vert_aabb.diagonal().norm();
}

void GlobalContactManager::update_contact_parameters()
{
    m_impl.update_contact_parameters();
}

Float GlobalContactManager::d_hat() const
{
    return m_impl.d_hat;
}
}  // namespace uipc::backend::cuda