#pragma once
#include <type_define.h>
#include <Eigen/Geometry>

namespace uipc::backend::cuda
{
using AABB = Eigen::AlignedBox<Float, 3>;
}  // namespace uipc::backend::cuda
