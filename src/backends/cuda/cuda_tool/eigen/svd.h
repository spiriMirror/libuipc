#pragma once
#include <type_define.h>
#include <cuda_tool/eigen/svd_impl.h>

namespace uipc::backend::cuda_tool
{
namespace eigen
{
    UIPC_GENERIC void svd(const Eigen::Matrix<float, 3, 3>& F,
                          Eigen::Matrix<float, 3, 3>&       U,
                          Eigen::Vector3<float>&            Sigma,
                          Eigen::Matrix<float, 3, 3>&       V);

    UIPC_GENERIC void pd(const Eigen::Matrix<float, 3, 3>& F,
                         Eigen::Matrix<float, 3, 3>&       R,
                         Eigen::Matrix<float, 3, 3>&       S);

    UIPC_GENERIC void svd(const Eigen::Matrix<double, 3, 3>& F,
                          Eigen::Matrix<double, 3, 3>&       U,
                          Eigen::Vector3<double>&            Sigma,
                          Eigen::Matrix<double, 3, 3>&       V);

    UIPC_GENERIC void pd(const Eigen::Matrix<double, 3, 3>& F,
                         Eigen::Matrix<double, 3, 3>&       R,
                         Eigen::Matrix<double, 3, 3>&       S);
}  // namespace eigen
}  // namespace uipc::backend::cuda_tool
#include "details/svd.inl"