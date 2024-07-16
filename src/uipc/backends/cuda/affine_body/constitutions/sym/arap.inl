// Function to extract F matrix from q vector
template <typename T>
__host__ __device__ void extractF(Eigen::Matrix<T, 3, 3>& F, const Eigen::Vector<T, 12>& q)
{
    F << q(3), q(4), q(5), q(6), q(7), q(8), q(9), q(10), q(11);
}

// Function to compute the rotation matrix R
template <typename T>
__host__ __device__ void computeR(Eigen::Matrix<T, 3, 3>&       R,
                                  const Eigen::Matrix<T, 3, 3>& F)
{
    Eigen::Matrix<T, 3, 3> U, V;
    Eigen::Vector3<T> Sigma;
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(F, Eigen::ComputeThinU | Eigen::ComputeThinV);
    U = svd.matrixU();
    V = svd.matrixV();
    R = U * V.transpose();
}

// Function to compute the ARAP energy
template <typename T>
__host__ __device__ void E(T& energy, const T& kappa, const T& v, const Eigen::Vector<T, 12>& q)
{
    Eigen::Matrix<T, 3, 3> F;
    extractF(F, q);
    Eigen::Matrix<T, 3, 3> R;
    Eigen::Matrix<T, 3, 3> U, V;
    Eigen::Vector3<T>      Sigma;
    muda::eigen::svd(F, U, Sigma, V);
    R = U * V.transpose();
    //computeR(R, F);
    energy = kappa * v * (F-R).squaredNorm();

}

// Function to compute the gradient of the ARAP energy
template <typename T>
__host__ __device__ void dEdq(Eigen::Vector<T, 9>&        gradient,
                              const T&                    kappa,
                              const T&                    v,
                              const Eigen::Vector<T, 12>& q)
{
    Eigen::Matrix<T, 3, 3> F;
    extractF(F, q);
    Eigen::Matrix<T, 3, 3> R;
    Eigen::Matrix<T, 3, 3> U, V;
    Eigen::Vector3<T>      Sigma;
    muda::eigen::svd(F, U, Sigma, V);
    R = U * V.transpose();

    Eigen::Matrix<T, 3, 3> dPsi_dF = 2 * (F - R);

    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            gradient(i * 3 + j)  =kappa * v * dPsi_dF(i, j);
        }
    }
}

// Function to flatten a matrix into a vector
template <typename T>
__host__ __device__ Eigen::Matrix<T, 9, 1> vec(const Eigen::Matrix<T, 3, 3>& mat)
{
    Eigen::Matrix<T, 9, 1> vector;
    vector << mat(0, 0), mat(1, 0), mat(2, 0), mat(0, 1), mat(1, 1), mat(2, 1),
        mat(0, 2), mat(1, 2), mat(2, 2);
    return vector;
}

// Function to compute the ARAP Hessian
template <typename T>
__host__ __device__ void ARAP_Hessian(Eigen::Matrix<T, 9, 9>&       H,
                                     const Eigen::Matrix<T, 3, 3>& F)
{
    Eigen::Matrix<T, 3, 3> U, V;
    Eigen::Vector3<T>      Sigma;
    muda::eigen::svd(F, U, Sigma, V);
    // Define the twist modes
    Eigen::Matrix<T, 3, 3> T0, T1, T2;
    T0 << 0, -1, 0, 1, 0, 0, 0, 0, 0;
    T1 << 0, 0, 0, 0, 0, 1, 0, -1, 0;
    T2 << 0, 0, 1, 0, 0, 0, -1, 0, 0;

    T0 = (1 / sqrt(2)) * U * T0 * V.transpose();
    T1 = (1 / sqrt(2)) * U * T1 * V.transpose();
    T2 = (1 / sqrt(2)) * U * T2 * V.transpose();

    // Flatten the twist modes
    Eigen::Matrix<T, 9, 1> t0 = vec(T0);
    Eigen::Matrix<T, 9, 1> t1 = vec(T1);
    Eigen::Matrix<T, 9, 1> t2 = vec(T2);

    // Get the singular values
    T s0 = Sigma(0);
    T s1 = Sigma(1);
    T s2 = Sigma(2);

    // Compute the Hessian
    H.setIdentity();
    return;
    H *= 2;
    H -= (4 / (s0 + s1)) * (t0 * t0.transpose());
    H -= (4 / (s1 + s2)) * (t1 * t1.transpose());
    H -= (4 / (s0 + s2)) * (t2 * t2.transpose());
}
// Function to compute the Hessian of the ARAP energy
template <typename T>
__host__ __device__ void ddEddq(Eigen::Matrix<T, 9, 9>&     hessian,
                                const T&                    kappa,
                                const T&                    v,
                                const Eigen::Vector<T, 12>& q)
{
    Eigen::Matrix<T, 3, 3> F;
    extractF(F, q);
    ARAP_Hessian(hessian, F);
    hessian *= kappa * v;
}