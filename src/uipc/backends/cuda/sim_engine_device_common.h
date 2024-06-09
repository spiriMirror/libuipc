#pragma once
#include <uipc/backend/macro.h>
#include <uipc/backends/cuda/sim_engine.h>
#include <muda/logger/logger.h>

namespace uipc::backend::cuda
{
class SimEngine::DeviceCommon
{
  public:
    U<muda::Logger> logger;
};
}  // namespace uipc::backend::cuda
