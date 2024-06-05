#pragma once
#include <muda/logger.h>

namespace uipc::backend
{
extern __device__ muda::LoggerViewer cout;

muda::LoggerViewer*                  device_logger_viewer_ptr() noexcept;
}
