namespace uipc::backend::cuda::distance
{
template <typename T>
MUDA_GENERIC void point_point_distance2(const Eigen::Vector<T, 3>& a,
                                        const Eigen::Vector<T, 3>& b,
                                        T&                         dist2)
{
    dist2 = (a - b).squaredNorm();
}

template <typename T>
MUDA_GENERIC void point_point_distance2_gradient(const Eigen::Vector<T, 3>& a,
                                                 const Eigen::Vector<T, 3>& b,
                                                 Eigen::Vector<T, 6>& grad)
{
    grad.template segment<3>(0) = 2.0 * (a - b);
    grad.template segment<3>(3) = -grad.template segment<3>(0);
}

template <typename T>
MUDA_GENERIC void point_point_distance2_hessian(const Eigen::Vector<T, 3>& a,
                                                const Eigen::Vector<T, 3>& b,
                                                Eigen::Matrix<T, 6, 6>& Hessian)
{
    Hessian.setZero();
    Hessian.diagonal().setConstant(2.0);
    Hessian(0, 3) = Hessian(1, 4) = Hessian(2, 5) = Hessian(3, 0) =
        Hessian(4, 1) = Hessian(5, 2) = -2.0;
}
}  // namespace uipc::backend::cuda::distance