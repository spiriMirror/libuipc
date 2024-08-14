template <typename T>
__host__ __device__ void FrictionEnergy(T& R, const T& coefficient, const T& eps_v, const T& h_hat, const T& y)
{
    T scalar = eps_v * h_hat;
    if (0 <= y && y < scalar) {
        R = coefficient * (-y * y * y / (3 * scalar * scalar) + y * y / scalar + scalar / 3);
    } else if (y >= scalar) {
        R = coefficient * y;
    } else {
        MUDA_ASSERT(false);
    }
}

template <typename T>
__host__ __device__ void dFrictionEnergydV(Eigen::Vector<T, 6>& R, const T& coefficient, const Eigen::Matrix<T, 6, 3>& Tk, const T& eps_v, const T& h_hat, const Eigen::Vector<T, 3>& vk)
{
    T y = vk.norm();
    Eigen::Vector<T, 3> s = vk / y; 
    if (0 < y && y < eps_v) {
        T f = - y * y / (eps_v * eps_v) + 2 * y / eps_v;
        R = coefficient * Tk * f * s * h_hat;
    }
    else if (y >= eps_v) {
        R = coefficient * Tk * s * h_hat;
    } else {
        R = Eigen::Vector<T, 6>::Zero();
    }
}

template <typename T>
__host__ __device__ void ddFrictionEnergyddV(Eigen::Matrix<T, 6, 6>& R, const T& coefficient, const Eigen::Matrix<T, 6, 3>& Tk, const T& eps_v, const T& h_hat, const Eigen::Vector<T, 3>& vk)
{
    T y = vk.norm();
    Eigen::Vector<T, 3> s = vk / y; 
    if (0 < y && y < eps_v) {
        T f = - y / (eps_v * eps_v) + 2 / eps_v;
        Eigen::Matrix<T, 3, 3> M = -vk * vk.transpose() / (eps_v * eps_v * y) + Eigen::Matrix<T, 3, 3>::Identity() * f;
        R = coefficient * Tk * M * Tk.transpose() * h_hat;
    } else if (y >= eps_v) {
        T f = 1 / y;
        Eigen::Matrix<T, 3, 3> M = -vk * vk.transpose() / (y * y * y) + Eigen::Matrix<T, 3, 3>::Identity() * f;
        R = coefficient * Tk * M * Tk.transpose() * h_hat;
    } else if (y == 0) {
        T f = 2 / eps_v;
        Eigen::Matrix<T, 3, 3> M = Eigen::Matrix<T, 3, 3>::Identity() * f;
        R = coefficient * Tk * M * Tk.transpose() * h_hat;
    } else {
        R = Eigen::Matrix<T, 6, 6>::Zero();
    }
}