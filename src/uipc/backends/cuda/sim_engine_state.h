#pragma once

namespace uipc::backend::cuda
{
enum class SimEngineState
{
    None = 0,
    RegisterSystems,
    BuildSystems,
    InitScene,
    RebuildScene,
    PredictMotion,
    ComputeGradientHassian,
    SolveGlobalLinearSystem,
    LineSearch,
    UpdateVelocity,
};
}
