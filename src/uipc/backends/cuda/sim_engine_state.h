#pragma once

namespace uipc::backend::cuda
{
enum class SimEngineState
{
    None         = 0,
    BuildSystems = 1,
    InitScene    = 2,
};
}
