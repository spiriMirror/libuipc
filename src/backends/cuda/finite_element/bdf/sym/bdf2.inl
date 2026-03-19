// BDF2 finite element helper functions.

template <typename T>
__host__ __device__ void compute_x_tilde(Eigen::Vector<T, 3>&       R,
                                         const Eigen::Vector<T, 3>& x_n,
                                         const Eigen::Vector<T, 3>& x_n_1,
                                         const Eigen::Vector<T, 3>& v_n,
                                         const Eigen::Vector<T, 3>& v_n_1,
                                         const T&                   h)
{
    const T c_xn   = static_cast<T>(4.0 / 3.0);
    const T c_xn_1 = static_cast<T>(1.0 / 3.0);
    const T c_v    = static_cast<T>(2.0 / 9.0) * h;

    R = c_xn * x_n - c_xn_1 * x_n_1 + c_v * (static_cast<T>(4.0) * v_n - v_n_1);
}

template <typename T>
__host__ __device__ void compute_x_tilde(Eigen::Vector<T, 3>&       R,
                                         const Eigen::Vector<T, 3>& x_n,
                                         const Eigen::Vector<T, 3>& x_n_1,
                                         const Eigen::Vector<T, 3>& v_n,
                                         const Eigen::Vector<T, 3>& v_n_1,
                                         const Eigen::Vector<T, 3>& g,
                                         const T&                   h)
{
    const T c_xn   = static_cast<T>(4.0 / 3.0);
    const T c_xn_1 = static_cast<T>(1.0 / 3.0);
    const T c_v    = static_cast<T>(2.0 / 9.0) * h;
    const T c_g    = static_cast<T>(4.0 / 9.0) * h * h;

    R = c_g * g + c_xn * x_n - c_xn_1 * x_n_1 + c_v * (static_cast<T>(4.0) * v_n - v_n_1);
}

template <typename T>
__host__ __device__ void compute_v(Eigen::Vector<T, 3>&       R,
                                   const Eigen::Vector<T, 3>& x,
                                   const Eigen::Vector<T, 3>& x_n,
                                   const Eigen::Vector<T, 3>& x_n_1,
                                   const T&                   h)
{
    const T c = static_cast<T>(0.5) / h;
    R         = c * (static_cast<T>(3.0) * x - static_cast<T>(4.0) * x_n + x_n_1);
}
