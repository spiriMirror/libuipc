
template <typename T>
__host__ __device__ void E(T& R, const T& mu, const T& lambda, const Eigen::Matrix<T, 3, 3>& F)
{

    auto J = F.determinant();
    auto Ic = F.squaredNorm();
    auto alpha = 1 + 0.75 * mu / lambda;
    R = 0.5 * lambda * (J - alpha) * (J - alpha) + 0.5 * mu * (Ic - 3) - 0.5 * mu * log(Ic + 1);
}

template <typename T>
__host__ __device__ void dEdVecF(Eigen::Matrix<T, 3, 3>& PEPF, const T& mu, const T& lambda, const Eigen::Matrix<T, 3, 3>& F)
{
    auto J  = F.determinant();
    auto Ic = F.squaredNorm();
    Eigen::Matrix<T, 3, 3> pJpF;

    pJpF(0, 0) = F(1, 1) * F(2, 2) - F(1, 2) * F(2, 1);
    pJpF(0, 1) = F(1, 2) * F(2, 0) - F(1, 0) * F(2, 2);
    pJpF(0, 2) = F(1, 0) * F(2, 1) - F(1, 1) * F(2, 0);

    pJpF(1, 0) = F(2, 1) * F(0, 2) - F(2, 2) * F(0, 1);
    pJpF(1, 1) = F(2, 2) * F(0, 0) - F(2, 0) * F(0, 2);
    pJpF(1, 2) = F(2, 0) * F(0, 1) - F(2, 1) * F(0, 0);

    pJpF(2, 0) = F(0, 1) * F(1, 2) - F(1, 1) * F(0, 2);
    pJpF(2, 1) = F(0, 2) * F(1, 0) - F(0, 0) * F(1, 2);
    pJpF(2, 2) = F(0, 0) * F(1, 1) - F(0, 1) * F(1, 0);

    PEPF = mu * (1 - 1 / (Ic + 1)) * F + (lambda * (J - 1 - 0.75 * mu / lambda)) * pJpF;
}

template <typename T>
__host__ __device__ void ddEddVecF(Eigen::Matrix<T, 9, 9>& R, const T& mu, const T& lambda, const Eigen::Matrix<T, 3, 3>& F)
{
    auto J  = F.determinant();
    auto Ic = F.squaredNorm();
    Eigen::Matrix<T, 9, 9> H1 = 2 * Eigen::Matrix<T, 9, 9>::Identity();
    Eigen::Matrix<T, 9, 1> g1;
    g1.block<3, 1>(0, 0) = 2 * F.col(0);
    g1.block<3, 1>(3, 0) = 2 * F.col(1);
    g1.block<3, 1>(6, 0) = 2 * F.col(2);
    Eigen::Matrix<T, 9, 1> gJ;
    gJ.block<3, 1>(0, 0) = F.col(1).cross(F.col(2));
    gJ.block<3, 1>(3, 0) = F.col(2).cross(F.col(0));
    gJ.block<3, 1>(6, 0) = F.col(0).cross(F.col(1));
    Eigen::Matrix<T, 3, 3> f0hat;
    f0hat << 0, -F(2, 0), F(1, 0), F(2, 0), 0, -F(0, 0), -F(1, 0), F(0, 0), 0;
    Eigen::Matrix<T, 3, 3> f1hat;
    f1hat << 0, -F(2, 1), F(1, 1), F(2, 1), 0, -F(0, 1), -F(1, 1), F(0, 1), 0;
    Eigen::Matrix<T, 3, 3> f2hat;
    f2hat << 0, -F(2, 2), F(1, 2), F(2, 2), 0, -F(0, 2), -F(1, 2), F(0, 2), 0;
    Eigen::Matrix<T, 9, 9> HJ;
    HJ.block<3, 3>(0, 0) = Eigen::Matrix<T, 3, 3>::Zero();
    HJ.block<3, 3>(0, 3) = -f2hat;
    HJ.block<3, 3>(0, 6) = f1hat;
    HJ.block<3, 3>(3, 0) = f2hat;
    HJ.block<3, 3>(3, 3) = Eigen::Matrix<T, 3, 3>::Zero();
    HJ.block<3, 3>(3, 6) = -f0hat;
    HJ.block<3, 3>(6, 0) = -f1hat;
    HJ.block<3, 3>(6, 3) = f0hat;
    HJ.block<3, 3>(6, 6) = Eigen::Matrix<T, 3, 3>::Zero();
    R = (Ic * mu) / (2 * (Ic + 1)) * H1 + lambda * (J - 1 - (3 * mu) / (4.0 * lambda)) * HJ + (mu / (2 * (Ic + 1) * (Ic + 1))) * g1 * g1.transpose() + lambda * gJ * gJ.transpose();
}
