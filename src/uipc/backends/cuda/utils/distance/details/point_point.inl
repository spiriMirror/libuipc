namespace uipc::backend::cuda::distance
{
template <class T, int dim>
MUDA_GENERIC void point_point_distance(const Eigen::Vector<T, dim>& a,
                                       const Eigen::Vector<T, dim>& b,
                                       T&                           dist2)
{
    dist2 = (a - b).squaredNorm();
}

template <class T, int dim>
MUDA_GENERIC void point_point_distance_gradient(const Eigen::Vector<T, dim>& a,
                                                const Eigen::Vector<T, dim>& b,
                                                Eigen::Vector<T, dim * 2>& grad)
{
    grad.template segment<dim>(0)   = 2.0 * (a - b);
    grad.template segment<dim>(dim) = -grad.template segment<dim>(0);
}

template <class T, int dim>
MUDA_GENERIC void point_point_distance_hessian(const Eigen::Vector<T, dim>& a,
                                               const Eigen::Vector<T, dim>& b,
                                               Eigen::Matrix<T, dim * 2, dim * 2>& Hessian)
{
    Hessian.setZero();
    Hessian.diagonal().setConstant(2.0);
    if constexpr(dim == 2)
    {
        Hessian(0, 2) = Hessian(1, 3) = Hessian(2, 0) = Hessian(3, 1) = -2.0;
    }
    else
    {
        Hessian(0, 3) = Hessian(1, 4) = Hessian(2, 5) = Hessian(3, 0) =
            Hessian(4, 1) = Hessian(5, 2) = -2.0;
    }
}

}  // namespace muda::distance