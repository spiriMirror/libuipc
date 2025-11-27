#pragma once
#include <type_define.h>
#include <Eigen/Geometry>

namespace uipc::backend::cuda
{
// float based AABB
using AABB = Eigen::AlignedBox<float, 3>;
}  // namespace uipc::backend::cuda
