// BDF2 affine body helper functions.

template <typename T>
__host__ __device__ void compute_q_tilde(Eigen::Vector<T, 12>&       R,
                                         const Eigen::Vector<T, 12>& q_n,
                                         const Eigen::Vector<T, 12>& q_n_1,
                                         const Eigen::Vector<T, 12>& qv_n,
                                         const Eigen::Vector<T, 12>& qv_n_1,
                                         const T&                    h)
{
    const T c_qn   = static_cast<T>(4.0 / 3.0);
    const T c_qn_1 = static_cast<T>(1.0 / 3.0);
    const T c_v    = static_cast<T>(2.0 / 9.0) * h;

    R = c_qn * q_n - c_qn_1 * q_n_1 + c_v * (static_cast<T>(4.0) * qv_n - qv_n_1);
}

template <typename T>
__host__ __device__ void compute_q_tilde(Eigen::Vector<T, 12>&       R,
                                         const Eigen::Vector<T, 12>& q_n,
                                         const Eigen::Vector<T, 12>& q_n_1,
                                         const Eigen::Vector<T, 12>& qv_n,
                                         const Eigen::Vector<T, 12>& qv_n_1,
                                         const Eigen::Vector<T, 12>& g,
                                         const T&                    h)
{
    const T c_qn   = static_cast<T>(4.0 / 3.0);
    const T c_qn_1 = static_cast<T>(1.0 / 3.0);
    const T c_v    = static_cast<T>(2.0 / 9.0) * h;
    const T c_g    = static_cast<T>(4.0 / 9.0) * h * h;

    R = c_g * g + c_qn * q_n - c_qn_1 * q_n_1 + c_v * (static_cast<T>(4.0) * qv_n - qv_n_1);
}

template <typename T>
__host__ __device__ void compute_qv(Eigen::Vector<T, 12>&       R,
                                    const Eigen::Vector<T, 12>& q,
                                    const Eigen::Vector<T, 12>& q_n,
                                    const Eigen::Vector<T, 12>& q_n_1,
                                    const T&                    h)
{
    const T c = static_cast<T>(0.5) / h;
    R         = c * (static_cast<T>(3.0) * q - static_cast<T>(4.0) * q_n + q_n_1);
}
