#pragma once
#include <uipc/backend/macro.h>
#include <uipc/backends/cuda/cuda_engine.h>
#include <muda/logger/logger.h>
namespace uipc::backend
{
class CudaEngine::DeviceCommon
{
  public:
    U<muda::Logger> logger;
};
}  // namespace uipc::backend
