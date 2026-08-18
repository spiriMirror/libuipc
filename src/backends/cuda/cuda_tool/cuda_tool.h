#pragma once
// cuda_tool: minimal raw-CUDA utilities used by the uipc CUDA backend.
// Drop-in replacement for the muda dependency (buffer/view/launch/cub/eigen/
// linear-system formats/debug). Namespace: uipc::backend::cuda_tool.
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>
#include <cuda_tool/view_nd.h>
#include <cuda_tool/launch.h>
#include <cuda_tool/buffer.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/eigen.h>
#include <cuda_tool/linear_system.h>
#include <cuda_tool/debug.h>
#include <cuda_tool/logger.h>
#include <cuda_tool/launch_base.h>
