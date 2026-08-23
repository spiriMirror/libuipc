#include <Eigen/Dense>
#include <algorithm/qr_svd.hpp>

namespace uipc::backend::cuda_tool::eigen
{
namespace details
{
    // Enforce the signed-SVD convention: proper rotations U,V (det = +1)
    // with a possible reflection absorbed into the smallest singular value
    // Sigma[2] (which may go negative when det(F) < 0).
    template <typename T>
    UIPC_INLINE UIPC_GENERIC void signed_svd_fixup(Eigen::Matrix<T, 3, 3>& U,
                                                   Eigen::Vector3<T>& Sigma,
                                                   Eigen::Matrix<T, 3, 3>& V)
    {
        using mat3 = Eigen::Matrix<T, 3, 3>;
        mat3 L     = mat3::Identity();
        L(2, 2)    = (U * V.transpose()).determinant();

        const T detU = U.determinant();
        const T detV = V.determinant();

        if(detU < 0.0 && detV > 0)
            U = U * L;
        if(detU > 0.0 && detV < 0.0)
            V = V * L;
        Sigma[2] = Sigma[2] * L(2, 2);
    }
}  // namespace details

UIPC_INLINE UIPC_GENERIC void svd(const Eigen::Matrix<float, 3, 3>& F,
                                  Eigen::Matrix<float, 3, 3>&       U,
                                  Eigen::Vector3<float>&            Sigma,
                                  Eigen::Matrix<float, 3, 3>&       V)
{
    // native QR-SVD (algorithm/qr_svd.hpp) — no Eigen JacobiSVD dependency
    uipc::backend::cuda::math::qr_svd(F, Sigma, U, V);
    details::signed_svd_fixup(U, Sigma, V);
}

UIPC_INLINE UIPC_GENERIC void pd(const Eigen::Matrix<float, 3, 3>& F,
                                 Eigen::Matrix<float, 3, 3>&       R,
                                 Eigen::Matrix<float, 3, 3>&       S)
{
    Eigen::Matrix<float, 3, 3> U, V;
    Eigen::Vector3<float>      Sigma;
    svd(F, U, Sigma, V);
    R = U * V.transpose();
    S = V * Sigma.asDiagonal() * V.transpose();
}

UIPC_INLINE UIPC_GENERIC void svd(const Eigen::Matrix<double, 3, 3>& F,
                                  Eigen::Matrix<double, 3, 3>&       U,
                                  Eigen::Vector3<double>&            Sigma,
                                  Eigen::Matrix<double, 3, 3>&       V)
{
    // native double precision (the previous implementation downcast to float)
    uipc::backend::cuda::math::qr_svd(F, Sigma, U, V);
    details::signed_svd_fixup(U, Sigma, V);
}

UIPC_INLINE UIPC_GENERIC void pd(const Eigen::Matrix<double, 3, 3>& F,
                                 Eigen::Matrix<double, 3, 3>&       R,
                                 Eigen::Matrix<double, 3, 3>&       S)
{
    Eigen::Matrix<double, 3, 3> U, V;
    Eigen::Vector3<double>      Sigma;
    svd(F, U, Sigma, V);
    R = U * V.transpose();
    S = V * Sigma.asDiagonal() * V.transpose();
}

}  // namespace uipc::backend::cuda_tool::eigen
