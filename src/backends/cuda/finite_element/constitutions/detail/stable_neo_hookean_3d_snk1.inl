// Stiff-GIPC SNK1 constitution (ported from StiffGIPC/femEnergy.cu):
//   energy   = __cal_StabbleNHK_energy1_3D
//   gradient = __computePEPF_StableNHK3D1_double
//   Hessian  = __project_StabbleNHK_H_3D1_makePD (analytic SPD projection)
//
// Unit-volume energy:  Psi = 0.5*mu*(Ic - 3) + 0.5*lambda*(J - 1 - mu/lambda)^2
// with Ic = |F|_F^2, J = det(F). The Hessian uses Stiff's analytic
// eigensystem (twist/flip modes from the SVD of F + a 3x3 direct eigensolve)
// instead of a generic 9x9 EVD + clamp.
// Included inside namespace uipc::backend::cuda::snk1 by
// stable_neo_hookean_3d_function.h — no includes allowed here.

template <typename T>
__host__ __device__ void E(T& R, const T& mu, const T& lambda, const Eigen::Matrix<T, 3, 3>& F)
{
    T J  = F.determinant();
    T Ic = F.squaredNorm();
    T Jm = J - 1 - mu / lambda;
    R    = 0.5 * (mu * (Ic - 3) + lambda * Jm * Jm);
}

template <typename T>
__host__ __device__ void dEdF(Eigen::Matrix<T, 3, 3>&       PEPF,
                              const T&                      mu,
                              const T&                      lambda,
                              const Eigen::Matrix<T, 3, 3>& F)
{
    T                      J = F.determinant();
    Eigen::Matrix<T, 3, 3> pJpF;  // cofactor matrix dJ/dF

    pJpF(0, 0) = F(1, 1) * F(2, 2) - F(1, 2) * F(2, 1);
    pJpF(0, 1) = F(1, 2) * F(2, 0) - F(1, 0) * F(2, 2);
    pJpF(0, 2) = F(1, 0) * F(2, 1) - F(1, 1) * F(2, 0);

    pJpF(1, 0) = F(2, 1) * F(0, 2) - F(2, 2) * F(0, 1);
    pJpF(1, 1) = F(2, 2) * F(0, 0) - F(2, 0) * F(0, 2);
    pJpF(1, 2) = F(2, 0) * F(0, 1) - F(2, 1) * F(0, 0);

    pJpF(2, 0) = F(0, 1) * F(1, 2) - F(1, 1) * F(0, 2);
    pJpF(2, 1) = F(0, 2) * F(1, 0) - F(0, 0) * F(1, 2);
    pJpF(2, 2) = F(0, 0) * F(1, 1) - F(0, 1) * F(1, 0);

    PEPF = mu * F + (lambda * (J - 1) - mu) * pJpF;
}

template <typename T>
__host__ __device__ void build_twist_flip_eigenvectors(const Eigen::Matrix<T, 3, 3>& U,
                                                       const Eigen::Matrix<T, 3, 3>& V,
                                                       Eigen::Matrix<T, 9, 9>& Q)
{
    const T                      scale = T(1) / sqrt(T(2));
    const Eigen::Matrix<T, 3, 3> sV    = scale * V;

    using M3 = Eigen::Matrix<T, 3, 3, Eigen::ColMajor>;

    M3 A;
    A << sV(0, 2) * U(0, 1), sV(1, 2) * U(0, 1), sV(2, 2) * U(0, 1),
        sV(0, 2) * U(1, 1), sV(1, 2) * U(1, 1), sV(2, 2) * U(1, 1),
        sV(0, 2) * U(2, 1), sV(1, 2) * U(2, 1), sV(2, 2) * U(2, 1);

    M3 B;
    B << sV(0, 1) * U(0, 2), sV(1, 1) * U(0, 2), sV(2, 1) * U(0, 2),
        sV(0, 1) * U(1, 2), sV(1, 1) * U(1, 2), sV(2, 1) * U(1, 2),
        sV(0, 1) * U(2, 2), sV(1, 1) * U(2, 2), sV(2, 1) * U(2, 2);

    M3 C;
    C << sV(0, 2) * U(0, 0), sV(1, 2) * U(0, 0), sV(2, 2) * U(0, 0),
        sV(0, 2) * U(1, 0), sV(1, 2) * U(1, 0), sV(2, 2) * U(1, 0),
        sV(0, 2) * U(2, 0), sV(1, 2) * U(2, 0), sV(2, 2) * U(2, 0);

    M3 D;
    D << sV(0, 0) * U(0, 2), sV(1, 0) * U(0, 2), sV(2, 0) * U(0, 2),
        sV(0, 0) * U(1, 2), sV(1, 0) * U(1, 2), sV(2, 0) * U(1, 2),
        sV(0, 0) * U(2, 2), sV(1, 0) * U(2, 2), sV(2, 0) * U(2, 2);

    M3 E;
    E << sV(0, 1) * U(0, 0), sV(1, 1) * U(0, 0), sV(2, 1) * U(0, 0),
        sV(0, 1) * U(1, 0), sV(1, 1) * U(1, 0), sV(2, 1) * U(1, 0),
        sV(0, 1) * U(2, 0), sV(1, 1) * U(2, 0), sV(2, 1) * U(2, 0);

    M3 F_;
    F_ << sV(0, 0) * U(0, 1), sV(1, 0) * U(0, 1), sV(2, 0) * U(0, 1),
        sV(0, 0) * U(1, 1), sV(1, 0) * U(1, 1), sV(2, 0) * U(1, 1),
        sV(0, 0) * U(2, 1), sV(1, 0) * U(2, 1), sV(2, 0) * U(2, 1);

    // Twist eigenvectors
    Eigen::Map<M3>(Q.data())      = B - A;
    Eigen::Map<M3>(Q.data() + 9)  = D - C;
    Eigen::Map<M3>(Q.data() + 18) = F_ - E;

    // Flip eigenvectors
    Eigen::Map<M3>(Q.data() + 27) = A + B;
    Eigen::Map<M3>(Q.data() + 36) = C + D;
    Eigen::Map<M3>(Q.data() + 45) = E + F_;
}

// Analytic SPD-projected 9x9 energy Hessian (replaces generic EVD + clamp).
template <typename T>
__host__ __device__ void ddEddF_spd(Eigen::Matrix<T, 9, 9>&       H,
                                    const T&                      mu,
                                    const T&                      lambda,
                                    const Eigen::Matrix<T, 3, 3>& F)
{
    using M3 = Eigen::Matrix<T, 3, 3, Eigen::ColMajor>;

    const T J = F.determinant();

    Eigen::Matrix<T, 3, 3> U, V;
    Eigen::Matrix<T, 3, 1> S;
    math::qr_svd(F, S, U, V);

    Eigen::Matrix<T, 9, 1> eigenvalues;
    Eigen::Matrix<T, 9, 9> Q;

    // Twist & flip eigenvalues: mu +/- sigma_i * (lambda*(J-1) - mu)
    const T evScale                    = lambda * (J - 1.0) - mu;
    eigenvalues.template segment<3>(0) = S * evScale;
    eigenvalues.template segment<3>(3) = -S * evScale;
    eigenvalues.template segment<6>(0).array() += mu;

    build_twist_flip_eigenvectors(U, V, Q);

    // Remaining three eigenpairs from the 3x3 stretch block
    {
        Eigen::Matrix<T, 3, 3> A;
        const T                s0s0 = S(0) * S(0);
        const T                s1s1 = S(1) * S(1);
        const T                s2s2 = S(2) * S(2);
        A(0, 0)                     = mu + lambda * s1s1 * s2s2;
        A(1, 1)                     = mu + lambda * s0s0 * s2s2;
        A(2, 2)                     = mu + lambda * s0s0 * s1s1;
        const T evScale2            = lambda * (2.0 * J - 1.0) - mu;
        A(0, 1)                     = evScale2 * S(2);
        A(1, 0)                     = A(0, 1);
        A(0, 2)                     = evScale2 * S(1);
        A(2, 0)                     = A(0, 2);
        A(1, 2)                     = evScale2 * S(0);
        A(2, 1)                     = A(1, 2);

        Eigen::Matrix<T, 3, 1> block_values;
        Eigen::Matrix<T, 3, 3> block_vectors;
        cuda_tool::eigen::evd<T, 3>(A, block_values, block_vectors);
        eigenvalues.template segment<3>(6) = block_values;

        Eigen::Map<M3>(Q.data() + 54) =
            U * block_vectors.col(0).asDiagonal() * V.transpose();
        Eigen::Map<M3>(Q.data() + 63) =
            U * block_vectors.col(1).asDiagonal() * V.transpose();
        Eigen::Map<M3>(Q.data() + 72) =
            U * block_vectors.col(2).asDiagonal() * V.transpose();
    }

    for(int i = 0; i < 9; i++)
    {
        if(eigenvalues(i) < 0.0)
            eigenvalues(i) = 0.0;
    }
    H = Q * eigenvalues.asDiagonal() * Q.transpose();
}
