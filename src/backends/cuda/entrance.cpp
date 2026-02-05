#include <sim_engine.h>
#include <backends/common/module.h>

UIPC_BACKEND_API EngineInterface* uipc_create_engine(EngineCreateInfo* info)
{
    return new uipc::backend::cuda::SimEngine(info);
}

UIPC_BACKEND_API void uipc_destroy_engine(EngineInterface* engine)
{
    delete engine;
}

#include <magic_enum/magic_enum.hpp>
#include <magic_enum/magic_enum_flags.hpp>
#include <energy_component_flags.h>

namespace uipc::backend::cuda
{
void test()
{
    using namespace magic_enum::bitwise_operators;
    magic_enum::enum_flags_name(EnergyComponentFlags::All);  // -> "Directions::Up|Directions::Right"
}
}  // namespace uipc::backend::cuda