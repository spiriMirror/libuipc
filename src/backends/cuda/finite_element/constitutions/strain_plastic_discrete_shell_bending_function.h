#pragma once
#include <type_define.h>
#include <utils/dihedral_angle.h>
#include <cmath>

namespace uipc::backend::cuda
{
namespace sym::strain_plastic_discrete_shell_bending
{
#include "sym/discrete_shell_bending.inl"

    template <typename T>
    inline UIPC_GENERIC constexpr T pi()
    {
        return static_cast<T>(3.14159265358979323846264338327950288);
    }

    template <typename T>
    inline UIPC_GENERIC constexpr T plasticity_write_threshold()
    {
        return static_cast<T>(1e-6);
    }

    template <typename T>
    inline UIPC_GENERIC constexpr T dihedral_guard_eps()
    {
        return static_cast<T>(1e-12);
    }

    template <typename T>
    inline UIPC_GENERIC bool is_finite_scalar(T v)
    {
        return isfinite(v);
    }

    template <typename T>
    inline UIPC_GENERIC T abs_value(T v)
    {
        return v < T(0) ? -v : v;
    }

    template <typename T>
    inline UIPC_GENERIC bool is_finite_vec3(const Eigen::Matrix<T, 3, 1>& v)
    {
        return is_finite_scalar(v[0]) && is_finite_scalar(v[1]) && is_finite_scalar(v[2]);
    }

    template <typename T>
    inline UIPC_GENERIC T wrap_angle(T angle)
    {
        constexpr T Pi    = pi<T>();
        constexpr T TwoPi = static_cast<T>(2.0) * Pi;

        while(angle > Pi)
            angle -= TwoPi;

        while(angle < -Pi)
            angle += TwoPi;

        return angle;
    }

    template <typename T>
    inline UIPC_GENERIC T angle_delta(T theta, T theta_bar)
    {
        return wrap_angle(theta - theta_bar);
    }

    template <typename T>
    inline UIPC_GENERIC bool safe_dihedral_angle(const Eigen::Matrix<T, 3, 1>& v0,
                                                 const Eigen::Matrix<T, 3, 1>& v1,
                                                 const Eigen::Matrix<T, 3, 1>& v2,
                                                 const Eigen::Matrix<T, 3, 1>& v3,
                                                 T&                            theta)
    {
        if(!is_finite_vec3(v0) || !is_finite_vec3(v1) || !is_finite_vec3(v2)
           || !is_finite_vec3(v3))
            return false;

        const Eigen::Matrix<T, 3, 1> n1 = (v1 - v0).cross(v2 - v0);
        const Eigen::Matrix<T, 3, 1> n2 = (v2 - v3).cross(v1 - v3);

        const T n1_sq = n1.squaredNorm();
        const T n2_sq = n2.squaredNorm();
        const T eps   = dihedral_guard_eps<T>();

        if(!is_finite_scalar(n1_sq) || !is_finite_scalar(n2_sq) || n1_sq <= eps || n2_sq <= eps)
            return false;

        const T denom = sqrt(n1_sq * n2_sq);
        if(!is_finite_scalar(denom) || denom <= eps)
            return false;

        T cos_theta = n1.dot(n2) / denom;
        if(!is_finite_scalar(cos_theta))
            return false;

        cos_theta = cos_theta < T(-1) ? T(-1) : cos_theta;
        cos_theta = cos_theta > T(1) ? T(1) : cos_theta;
        theta     = acos(cos_theta);
        if(!is_finite_scalar(theta))
            return false;

        if(n2.cross(n1).dot(v1 - v2) < 0)
            theta = -theta;

        return is_finite_scalar(theta);
    }

    template <typename T>
    inline UIPC_GENERIC bool update_plastic_state(T  theta,
                                                  T& theta_bar,
                                                  T& yield_threshold,
                                                  T  hardening_modulus)
    {
        if(!is_finite_scalar(theta) || !is_finite_scalar(theta_bar)
           || !is_finite_scalar(yield_threshold) || !is_finite_scalar(hardening_modulus)
           || yield_threshold < T(0) || hardening_modulus < T(0))
            return false;

        const T delta          = angle_delta(theta, theta_bar);
        const T plastic_excess = abs_value(delta) - yield_threshold;

        if(!is_finite_scalar(plastic_excess)
           || plastic_excess <= plasticity_write_threshold<T>())
            return false;

        const T direction = delta >= T(0) ? T(1) : T(-1);

        theta_bar       = wrap_angle(theta_bar + direction * plastic_excess);
        yield_threshold = yield_threshold + hardening_modulus * plastic_excess;

        return is_finite_scalar(theta_bar) && is_finite_scalar(yield_threshold)
               && yield_threshold >= T(0);
    }

    template <typename T>
    inline UIPC_GENERIC bool try_angle_delta(const Eigen::Matrix<T, 3, 1>& x0,
                                             const Eigen::Matrix<T, 3, 1>& x1,
                                             const Eigen::Matrix<T, 3, 1>& x2,
                                             const Eigen::Matrix<T, 3, 1>& x3,
                                             T                            theta_bar,
                                             T&                           theta,
                                             T&                           delta)
    {
        if(!is_finite_scalar(theta_bar))
            return false;

        if(!safe_dihedral_angle(x0, x1, x2, x3, theta))
            return false;

        delta = angle_delta(theta, theta_bar);
        return is_finite_scalar(delta);
    }

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
        Vector3 n2 = (x2_bar - x3_bar).cross(x1_bar - x3_bar);
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
        namespace PDSB = sym::strain_plastic_discrete_shell_bending;
        Float theta = 0.0;
        Float delta = 0.0;
        if(!PDSB::try_angle_delta(x0, x1, x2, x3, theta_bar, theta, delta))
            return 0.0;

        Float R;
        PDSB::E(R, kappa, theta_bar + delta, theta_bar, L0, h_bar);
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
        namespace PDSB = sym::strain_plastic_discrete_shell_bending;
        Float theta = 0.0;
        Float delta = 0.0;
        if(!PDSB::try_angle_delta(x0, x1, x2, x3, theta_bar, theta, delta))
        {
            G.setZero();
            return;
        }

        Float dEdtheta;
        PDSB::dEdtheta(dEdtheta, kappa, theta_bar + delta, theta_bar, L0, h_bar);

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
        namespace PDSB = sym::strain_plastic_discrete_shell_bending;
        Float theta = 0.0;
        Float delta = 0.0;
        if(!PDSB::try_angle_delta(x0, x1, x2, x3, theta_bar, theta, delta))
        {
            H.setZero();
            return;
        }

        Float dEdtheta;
        PDSB::dEdtheta(dEdtheta, kappa, theta_bar + delta, theta_bar, L0, h_bar);

        Float ddEddtheta;
        PDSB::ddEddtheta(ddEddtheta, kappa, theta_bar + delta, theta_bar, L0, h_bar);

        Vector12 dthetadx;
        dihedral_angle_gradient(x0, x1, x2, x3, dthetadx);

        Matrix12x12 ddthetaddx;
        dihedral_angle_hessian(x0, x1, x2, x3, ddthetaddx);

        H = dthetadx * ddEddtheta * dthetadx.transpose() + dEdtheta * ddthetaddx;
    }
}  // namespace sym::strain_plastic_discrete_shell_bending
}  // namespace uipc::backend::cuda
