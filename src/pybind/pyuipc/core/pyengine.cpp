#include <pyuipc/core/pyengine.h>
#include <uipc/core/internal/world.h>


namespace pyuipc::core
{
void PyIEngine::do_init(uipc::core::internal::World& w)
{
    this->m_world = uipc::make_shared<uipc::core::World>(w.shared_from_this());
    do_init();
};
}  // namespace pyuipc::core
