#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::baraff_witkin_shell_2d
{
    inline UIPC_GENERIC Vector6 flatten(const Matrix<Float, 3, 2>& F)
    {
        Vector6 R;
        R.segment<3>(0) = F.col(0);
        R.segment<3>(3) = F.col(1);
        return R;
    }

    inline UIPC_GENERIC Matrix<Float, 3, 2> F3x2(const Matrix<Float, 3, 2>& Ds3x2,
                                                 const Matrix2x2& Dms2x2_inv)
    {
        return Ds3x2 * Dms2x2_inv;
    }

    inline UIPC_GENERIC Matrix<Float, 3, 2> Ds3x2(const Vector3& x0,
                                                  const Vector3& x1,
                                                  const Vector3& x2)
    {
        Matrix<Float, 3, 2> M;
        M.col(0) = x1 - x0;
        M.col(1) = x2 - x0;
        return M;
    }


    inline UIPC_GENERIC Matrix2x2 Dm2x2(const Vector3& x0, const Vector3& x1, const Vector3& x2)
    {
        Vector3 v01 = x1 - x0;
        Vector3 v02 = x2 - x1;
        // compute uv coordinates by rotating each triangle normal to (0, 1, 0)
        Vector3 normal = v01.cross(v02).normalized();
        Vector3 target = Vector3(0, 1, 0);


        Vector3 vec      = normal.cross(target);
        Float           sin      = vec.norm();
        Float           cos      = normal.dot(target);
        Matrix3x3 rotation = Matrix3x3::Identity();
        Matrix3x3 cross_vec;

        cross_vec << 0, -vec.z(), vec.y(),  //
            vec.z(), 0, -vec.x(),           //
            -vec.y(), vec.x(), 0;

        rotation += cross_vec + cross_vec * cross_vec / (1 + cos);

        Vector3 rotate_uv0 = rotation * x0;
        Vector3 rotate_uv1 = rotation * x1;
        Vector3 rotate_uv2 = rotation * x2;

        auto      uv0 = Vector2(rotate_uv0.x(), rotate_uv0.z());
        auto      uv1 = Vector2(rotate_uv1.x(), rotate_uv1.z());
        auto      uv2 = Vector2(rotate_uv2.x(), rotate_uv2.z());
        Matrix2x2 M;
        M.col(0) = uv1 - uv0;
        M.col(1) = uv2 - uv0;
        return M;
    }

    inline UIPC_GENERIC Float E(const Matrix<Float, 3, 2>& F,
                                Float                      area_0,
                                const Vector2&             anisotropic_a,
                                const Vector2&             anisotropic_b,
                                Float                      stretchS,
                                Float                      shearS,
                                Float                      strainRate)
    {
        Float I6 = anisotropic_a.transpose() * F.transpose() * F * anisotropic_b;
        Float shear_energy = I6 * I6;


        Float I5u = (F * anisotropic_a).norm();
        Float I5v = (F * anisotropic_b).norm();

        Float ucoeff = 1;
        Float vcoeff = 1;

        if(I5u <= 1)
        {
            ucoeff = 0;
        }
        if(I5v <= 1)
        {
            vcoeff = 0;
        }


        Float stretch_energy =
            std::pow(I5u - 1, 2) + ucoeff * strainRate * std::pow(I5u - 1, 3)
            + std::pow(I5v - 1, 2) + vcoeff * strainRate * std::pow(I5v - 1, 3);
        return (stretchS * stretch_energy + shearS * shear_energy) * area_0;
    }

    inline UIPC_GENERIC void dEdF(Matrix<Float, 3, 2>        R,
                                  const Matrix<Float, 3, 2>& F,
                                  Float                      area_0,
                                  const Vector2&             anisotropic_a,
                                  const Vector2&             anisotropic_b,
                                  Float                      stretchS,
                                  Float                      shearS,
                                  Float                      strainRate)
    {
        Float I6 = anisotropic_a.transpose() * F.transpose() * F * anisotropic_b;
        Eigen::Matrix<Float, 3, 2> stretch_pk1, shear_pk1;

        shear_pk1 = 2 * (I6 - anisotropic_a.transpose() * anisotropic_b)
                    * (F * anisotropic_a * anisotropic_b.transpose()
                       + F * anisotropic_b * anisotropic_a.transpose());
        Float I5u    = (F * anisotropic_a).transpose() * F * anisotropic_a;
        Float I5v    = (F * anisotropic_b).transpose() * F * anisotropic_b;
        Float ucoeff = Float{1} - Float{1} / sqrt(I5u);
        Float vcoeff = Float{1} - Float{1} / sqrt(I5v);

        if(I5u > 1)
        {
            ucoeff += 1.5 * strainRate * (sqrt(I5u) + 1 / sqrt(I5u) - 2);
        }
        if(I5v > 1)
        {
            vcoeff += 1.5 * strainRate * (sqrt(I5v) + 1 / sqrt(I5v) - 2);
        }


        stretch_pk1 = ucoeff * Float{2} * F * anisotropic_a * anisotropic_a.transpose()
                      + vcoeff * Float{2} * F * anisotropic_b * anisotropic_b.transpose();

        R = (area_0 * stretchS * stretch_pk1 + area_0 * shearS * shear_pk1);
    }

    inline UIPC_GENERIC void ddEddF(Eigen::Matrix<Float, 6, 6>& R,
                                    const Matrix<Float, 3, 2>&  F,
                                    Float                       area_0,
                                    const Vector2&              anisotropic_a,
                                    const Vector2&              anisotropic_b,
                                    Float                       stretchS,
                                    Float                       shearS,
                                    Float                       strainRate)
    {
        Eigen::Matrix<Float, 6, 6> final_H = Eigen::Matrix<Float, 6, 6>::Zero();
        {
            Eigen::Matrix<Float, 6, 6> H;
            H.setZero();
            Float I5u = (F * anisotropic_a).transpose() * F * anisotropic_a;
            Float I5v = (F * anisotropic_b).transpose() * F * anisotropic_b;
            Float invSqrtI5u = Float{1} / sqrt(I5u);
            Float invSqrtI5v = Float{1} / sqrt(I5v);

            Float sqrtI5u = sqrt(I5u);
            Float sqrtI5v = sqrt(I5v);

            if(sqrtI5u > 1)
                H(0, 0) = H(1, 1) = H(2, 2) =
                    2
                    * (((sqrtI5u - 1) * (3 * sqrtI5u * strainRate - 3 * strainRate + 2))
                       / (2 * sqrtI5u));
            if(sqrtI5v > 1)
                H(3, 3) = H(4, 4) = H(5, 5) =
                    2
                    * (((sqrtI5v - 1) * (3 * sqrtI5v * strainRate - 3 * strainRate + 2))
                       / (2 * sqrtI5v));
            auto fu = F.col(0).normalized();
            auto fv = F.col(1).normalized();

            Float uCoeff = (sqrtI5u > Float{1.0}) ?
                               (3 * I5u * strainRate - 3 * strainRate + 2) / (sqrt(I5u)) :
                               2.0;
            Float vCoeff = (sqrtI5v > Float{1.0}) ?
                               (3 * I5v * strainRate - 3 * strainRate + 2) / (sqrt(I5v)) :
                               Float{2.0};


            H.block<3, 3>(0, 0) += uCoeff * (fu * fu.transpose());
            H.block<3, 3>(3, 3) += vCoeff * (fv * fv.transpose());

            final_H += (area_0 * stretchS) * H;
        }
        {
            Eigen::Matrix<Float, 6, 6> H_shear;
            H_shear.setZero();
            Eigen::Matrix<Float, 6, 6> H = Eigen::Matrix<Float, 6, 6>::Zero();
            H(3, 0) = H(4, 1) = H(5, 2) = H(0, 3) = H(1, 4) = H(2, 5) = 1.0;
            Float I6 = anisotropic_a.transpose() * F.transpose() * F * anisotropic_b;
            Float signI6 = (I6 >= 0) ? 1.0 : -1.0;
            auto  g      = F
                     * (anisotropic_a * anisotropic_b.transpose()
                        + anisotropic_b * anisotropic_a.transpose());
            Eigen::Matrix<Float, 6, 1> vec_g = Eigen::Matrix<Float, 6, 1>::Zero();

            vec_g.block(0, 0, 3, 1) = g.col(0);
            vec_g.block(3, 0, 3, 1) = g.col(1);
            Float I2                = F.squaredNorm();
            Float lambda0 = 0.5 * (I2 + sqrt(I2 * I2 + 12.0 * I6 * I6));
            Eigen::Matrix<Float, 6, 1> q0 =
                (I6 * H * vec_g + lambda0 * vec_g).normalized();
            Eigen::Matrix<Float, 6, 6> T = Eigen::Matrix<Float, 6, 6>::Identity();
            T            = Float{0.5} * (T + signI6 * H);
            auto  Tq     = T * q0;
            Float normTq = Tq.squaredNorm();
            H_shear      = fabs(I6) * (T - (Tq * Tq.transpose()) / normTq)
                      + lambda0 * (q0 * q0.transpose());
            H_shear *= 2;
            final_H += (area_0 * shearS) * H_shear;
        }
        R = final_H;
    }
}  // namespace sym::baraff_witkin_shell_2d
}  // namespace uipc::backend::cuda
