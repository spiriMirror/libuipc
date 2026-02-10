#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::external_revolute_joint_constraint
{
#include <affine_body/constraints/sym/external_articulation_revolute_joint_constraint.inl>
}

namespace sym::external_prismatic_joint_constraint
{
#include <affine_body/constraints/sym/external_articulation_prismatic_joint_constraint.inl>
}
}  // namespace uipc::backend::cuda