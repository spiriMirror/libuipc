#pragma once
#include <type_define.h>
#include <utils/dihedral_angle.h>
//ref: https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf
namespace uipc::backend::cuda
{
namespace sym::discrete_shell_bending
{
#include "sym/discrete_shell_bending.inl"

    inline UIPC_GENERIC void compute_constants(Float&         L0,
                                               Float&         h_bar,
                                               Float&         theta_bar,
                                               Float&         V_bar,
                                               const Vector3& x0_bar,
                                               const Vector3& x1_bar,
                                               const Vector3& x2_bar,
                                               const Vector3& x3_bar,
                                               Float          thickness0,
                                               Float          thickness1,
                                               Float          thickness2,
                                               Float          thickness3)

    {
        L0         = (x2_bar - x1_bar).norm();
        Vector3 n1 = (x1_bar - x0_bar).cross(x2_bar - x0_bar);
        Vector3 n2 = (x2_bar - x0_bar).cross(x1_bar - x3_bar);
        Float   A  = (n1.norm() + n2.norm()) / 2.0;
        h_bar      = A / 3.0 / L0;
        dihedral_angle(x0_bar, x1_bar, x2_bar, x3_bar, theta_bar);

        Float thickness = (thickness0 + thickness1 + thickness2 + thickness3) / 4.0;
        V_bar = A * thickness;
    }

    inline UIPC_GENERIC Float E(const Vector3& x0,
                                const Vector3& x1,
                                const Vector3& x2,
                                const Vector3& x3,
                                Float          L0,
                                Float          h_bar,
                                Float          theta_bar,
                                Float          kappa)
    {

        namespace DSB = sym::discrete_shell_bending;
        Float theta;
        dihedral_angle(x0, x1, x2, x3, theta);

        Float R;
        DSB::E(R, kappa, theta, theta_bar, L0, h_bar);

        return R;
    }

    inline UIPC_GENERIC void dEdx(Vector12&      G,
                                  const Vector3& x0,
                                  const Vector3& x1,
                                  const Vector3& x2,
                                  const Vector3& x3,
                                  Float          L0,
                                  Float          h_bar,
                                  Float          theta_bar,
                                  Float          kappa)
    {
        namespace DSB = sym::discrete_shell_bending;
        Float theta;
        dihedral_angle(x0, x1, x2, x3, theta);

        Float dEdtheta;
        DSB::dEdtheta(dEdtheta, kappa, theta, theta_bar, L0, h_bar);

        Vector12 dthetadx;
        dihedral_angle_gradient(x0, x1, x2, x3, dthetadx);

        G = dEdtheta * dthetadx;
    }

    inline UIPC_GENERIC void ddEddx(Matrix12x12&   H,
                                    const Vector3& x0,
                                    const Vector3& x1,
                                    const Vector3& x2,
                                    const Vector3& x3,
                                    Float          L0,
                                    Float          h_bar,
                                    Float          theta_bar,
                                    Float          kappa)
    {
        namespace DSB = sym::discrete_shell_bending;
        Float theta;
        dihedral_angle(x0, x1, x2, x3, theta);

        Float dEdtheta;
        DSB::dEdtheta(dEdtheta, kappa, theta, theta_bar, L0, h_bar);

        Float ddEddtheta;
        DSB::ddEddtheta(ddEddtheta, kappa, theta, theta_bar, L0, h_bar);

        Vector12 dthetadx;
        dihedral_angle_gradient(x0, x1, x2, x3, dthetadx);

        Matrix12x12 ddthetaddx;
        dihedral_angle_hessian(x0, x1, x2, x3, ddthetaddx);


        H = dthetadx * ddEddtheta * dthetadx.transpose() + dEdtheta * ddthetaddx;
    }

}  // namespace sym::discrete_shell_bending
}  // namespace uipc::backend::cuda
