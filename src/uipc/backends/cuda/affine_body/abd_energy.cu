#include <type_define.h>
#include <Eigen/Dense>

namespace uipc::backend::cuda
{
MUDA_GENERIC Float shape_energy(const Vector12& q)
{
    Float       ret = 0.0;
    const auto& a1  = q.segment<3>(3);
    const auto& a2  = q.segment<3>(6);
    const auto& a3  = q.segment<3>(9);

    constexpr auto square = [](auto x) { return x * x; };

    ret += square(a1.squaredNorm() - 1.0);
    ret += square(a2.squaredNorm() - 1.0);
    ret += square(a3.squaredNorm() - 1.0);

    ret += square(a1.dot(a2)) * 2;
    ret += square(a2.dot(a3)) * 2;
    ret += square(a3.dot(a1)) * 2;

    return ret;
}

MUDA_GENERIC Vector9 shape_energy_gradient(const Vector12& q)
{
    Vector9 ret;

    const auto& a1 = q.segment<3>(3);
    const auto& a2 = q.segment<3>(6);
    const auto& a3 = q.segment<3>(9);

    auto dEda1 = ret.segment<3>(0);
    auto dEda2 = ret.segment<3>(3);
    auto dEda3 = ret.segment<3>(6);

    dEda1 = 4.0 * (a1.squaredNorm() - 1.0) * a1 + 4.0 * a2.dot(a1) * a2
            + 4.0 * (a3.dot(a1)) * a3;

    dEda2 = 4.0 * (a2.squaredNorm() - 1.0) * a2 + 4.0 * a3.dot(a2) * a3
            + 4.0 * a1.dot(a2) * a1;

    dEda3 = 4.0 * (a3.squaredNorm() - 1.0) * a3 + 4.0 * a1.dot(a3) * a1
            + 4.0 * a2.dot(a3) * a2;

    return ret;
}


MUDA_GENERIC void ddV_ddai(Matrix3x3& ddV_ddai,
                           const Eigen::VectorBlock<const Vector12, 3>& ai,
                           const Eigen::VectorBlock<const Vector12, 3>& aj,
                           const Eigen::VectorBlock<const Vector12, 3>& ak)
{
    ddV_ddai = 8.0 * ai * ai.transpose()
               + 4.0 * (ai.squaredNorm() - 1) * Matrix3x3::Identity()
               + 4.0 * aj * aj.transpose() + 4.0 * ak * ak.transpose();
}

MUDA_GENERIC void ddV_daidaj(Matrix3x3& ddV_daidaj,
                             const Eigen::VectorBlock<const Vector12, 3>& ai,
                             const Eigen::VectorBlock<const Vector12, 3>& aj,
                             const Eigen::VectorBlock<const Vector12, 3>& ak)
{
    ddV_daidaj = 4.0 * aj * ai.transpose() + 4.0 * ai.dot(aj) * Matrix3x3::Identity();
}

MUDA_GENERIC void shape_energy_hessian(const Vector12& q,
                                       Matrix3x3&      ddVdda1,
                                       Matrix3x3&      ddVdda2,
                                       Matrix3x3&      ddVdda3,
                                       Matrix3x3&      ddVda1da2,
                                       Matrix3x3&      ddVda1da3,
                                       Matrix3x3&      ddVda2da3)
{
    const auto& a1 = q.segment<3>(3);
    const auto& a2 = q.segment<3>(6);
    const auto& a3 = q.segment<3>(9);

    ddV_ddai(ddVdda1, a1, a2, a3);
    ddV_ddai(ddVdda2, a2, a3, a1);
    ddV_ddai(ddVdda3, a3, a1, a2);

    ddV_daidaj(ddVda1da2, a1, a2, a3);
    ddV_daidaj(ddVda1da3, a1, a3, a2);
    ddV_daidaj(ddVda2da3, a2, a3, a1);
}

MUDA_GENERIC Matrix9x9 shape_energy_hessian(const Vector12& q)
{
    Matrix9x9 H = Matrix9x9::Zero();

    Matrix3x3 ddVdda1, ddVdda2, ddVdda3, ddVda1da2, ddVda1da3, ddVda2da3;
    shape_energy_hessian(q, ddVdda1, ddVdda2, ddVdda3, ddVda1da2, ddVda1da3, ddVda2da3);


    //tex:
    //$$
    //\begin{bmatrix}
    //   \frac{\partial^2 V}{\partial a_1^2} & \frac{\partial^2 V}{\partial a_1 \partial a_2} & \frac{\partial^2 V}{\partial a_1 \partial a_3} \\
    //   \frac{\partial^2 V}{\partial a_2 \partial a_1} & \frac{\partial^2 V}{\partial a_2^2} & \frac{\partial^2 V}{\partial a_2 \partial a_3} \\
    //   \frac{\partial^2 V}{\partial a_3 \partial a_1} & \frac{\partial^2 V}{\partial a_3 \partial a_2} & \frac{\partial^2 V}{\partial a_3^2} \\
    //\end{bmatrix}
    //$$

    H.block<3, 3>(0, 0) = ddVdda1;
    H.block<3, 3>(0, 3) = ddVda1da2;
    H.block<3, 3>(0, 6) = ddVda1da3;

    H.block<3, 3>(3, 0) = ddVda1da2.transpose();
    H.block<3, 3>(3, 3) = ddVdda2;
    H.block<3, 3>(3, 6) = ddVda2da3;

    H.block<3, 3>(6, 0) = ddVda1da3.transpose();
    H.block<3, 3>(6, 3) = ddVda2da3.transpose();
    H.block<3, 3>(6, 6) = ddVdda3;

    return H;
}
}  // namespace uipc::backend::cuda
