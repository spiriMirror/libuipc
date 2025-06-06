#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::affine_body_revolute_joint
{
    inline UIPC_GENERIC Float E(const Vector12& X)
    {
        Float E0 = (X.segment<3>(0) - X.segment<3>(6)).squaredNorm();
        Float E1 = (X.segment<3>(3) - X.segment<3>(9)).squaredNorm();
        return (E0 + E1) / 2;
    }

#include "sym/affine_body_revolute_joint.inl"
}  // namespace sym::affine_body_revolute_joint
}  // namespace uipc::backend::cuda