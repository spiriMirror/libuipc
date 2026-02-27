#ifdef __CUDA_ARCH__
#include <muda/ext/eigen/svd/svd_impl.h>

namespace muda::eigen
{
namespace details
{
    MUDA_INLINE MUDA_DEVICE void device_svd(const Eigen::Matrix<float, 3, 3>& F,
                                            Eigen::Matrix<float, 3, 3>&       U,
                                            Eigen::Vector3<float>&      Sigma,
                                            Eigen::Matrix<float, 3, 3>& V)
    {
        muda::details::eigen::svd3x3(F(0, 0),
                                     F(0, 1),
                                     F(0, 2),
                                     F(1, 0),
                                     F(1, 1),
                                     F(1, 2),
                                     F(2, 0),
                                     F(2, 1),
                                     F(2, 2),
                                     U(0, 0),
                                     U(0, 1),
                                     U(0, 2),
                                     U(1, 0),
                                     U(1, 1),
                                     U(1, 2),
                                     U(2, 0),
                                     U(2, 1),
                                     U(2, 2),
                                     Sigma(0),
                                     Sigma(1),
                                     Sigma(2),
                                     V(0, 0),
                                     V(0, 1),
                                     V(0, 2),
                                     V(1, 0),
                                     V(1, 1),
                                     V(1, 2),
                                     V(2, 0),
                                     V(2, 1),
                                     V(2, 2));
    }
}  // namespace details
}  // namespace muda::eigen
#endif

#include <muda/ext/eigen/eigen_dense_cxx20.h>
namespace muda::eigen
{
MUDA_INLINE MUDA_GENERIC void svd(const Eigen::Matrix<float, 3, 3>& F,
                                  Eigen::Matrix<float, 3, 3>&       U,
                                  Eigen::Vector3<float>&            Sigma,
                                  Eigen::Matrix<float, 3, 3>&       V)
{
    using mat3 = Eigen::Matrix<float, 3, 3>;
    using vec3 = Eigen::Vector3<float>;
#ifdef __CUDA_ARCH__
    details::device_svd(F, U, Sigma, V);
#else
    const Eigen::JacobiSVD<mat3, Eigen::NoQRPreconditioner> svd(
        F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U     = svd.matrixU();
    V     = svd.matrixV();
    Sigma = svd.singularValues();
#endif
    mat3 L  = mat3::Identity();
    L(2, 2) = (U * V.transpose()).determinant();

    const float detU = U.determinant();
    const float detV = V.determinant();

    if(detU < 0.0 && detV > 0)
        U = U * L;
    if(detU > 0.0 && detV < 0.0)
        V = V * L;
    Sigma[2] = Sigma[2] * L(2, 2);
}

MUDA_INLINE MUDA_GENERIC void pd(const Eigen::Matrix<float, 3, 3>& F,
                                 Eigen::Matrix<float, 3, 3>&       R,
                                 Eigen::Matrix<float, 3, 3>&       S)
{
    Eigen::Matrix<float, 3, 3> U, V;
    Eigen::Vector3<float>      Sigma;
    svd(F, U, Sigma, V);
    R = U * V.transpose();
    S = V * Sigma.asDiagonal() * V.transpose();
}

MUDA_INLINE MUDA_GENERIC void svd(const Eigen::Matrix<double, 3, 3>& F,
                      Eigen::Matrix<double, 3, 3>&       U,
                      Eigen::Vector3<double>&            Sigma,
                      Eigen::Matrix<double, 3, 3>&       V)
{
    Eigen::Matrix3f fU;
    Eigen::Vector3f fSigma;
    Eigen::Matrix3f fV;
    svd(F.cast<float>(), fU, fSigma, fV);
    U     = fU.cast<double>();
    Sigma = fSigma.cast<double>();
    V     = fV.cast<double>();
}

MUDA_INLINE MUDA_GENERIC void pd(const Eigen::Matrix<double, 3, 3>& F,
                     Eigen::Matrix<double, 3, 3>&       R,
                     Eigen::Matrix<double, 3, 3>&       S)
{
    Eigen::Matrix3f fR;
    Eigen::Matrix3f fS;
    pd(F.cast<float>(), fR, fS);
    R = fR.cast<double>();
    S = fS.cast<double>();
}

}  // namespace muda::eigen