#pragma once
#include <type_define.h>
#include <utils/dihedral_angle.h>

namespace uipc::backend::cuda::detail
{
inline UIPC_GENERIC void compute_discrete_shell_bending_reference(Float& L0,
                                                                  Float& h_bar,
                                                                  Float& theta_bar,
                                                                  Float& outer_weight,
                                                                  const Vector3& x0_bar,
                                                                  const Vector3& x1_bar,
                                                                  const Vector3& x2_bar,
                                                                  const Vector3& x3_bar)
{
    L0         = (x2_bar - x1_bar).norm();
    Vector3 n1 = (x1_bar - x0_bar).cross(x2_bar - x0_bar);
    Vector3 n2 = (x2_bar - x3_bar).cross(x1_bar - x3_bar);
    Float   A  = (n1.norm() + n2.norm()) / 2.0;
    h_bar      = A / 3.0 / L0;
    dihedral_angle(x0_bar, x1_bar, x2_bar, x3_bar, theta_bar);

    // L0 / h_bar already contains the complete Discrete Shells reference
    // metric (3 * L0^2 / A). Keep the generic outer multiplier neutral;
    // multiplying by A again would cancel the required area normalization.
    outer_weight = 1.0;
}
}  // namespace uipc::backend::cuda::detail
