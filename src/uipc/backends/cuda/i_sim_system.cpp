#include <i_sim_system.h>
#include <typeinfo>

namespace uipc::backend::cuda
{
std::string_view ISimSystem::get_name() const noexcept
{
    return typeid(*this).name();
}

std::string_view ISimSystem::name() const noexcept
{
    return get_name();
}
}  // namespace uipc::backend::cuda
