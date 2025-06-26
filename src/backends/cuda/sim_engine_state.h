#pragma once

namespace uipc::backend::cuda
{
enum class SimEngineState
{
    None = 0,
    BuildSystems,
    InitScene,
    RebuildScene,
    PredictMotion,
    ComputeContact,
    // ComputeGradientHessian,
    SolveGlobalLinearSystem,
    LineSearch,
    UpdateVelocity,
};
}
