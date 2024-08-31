#pragma once
#include <type_define.h>
#include <utils/distance.h>
#include <muda/ext/eigen/evd.h>
#include <contact_system/contact_models/codim_ipc_contact_function.h>
#include <utils/friction_utils.h>

namespace uipc::backend::cuda
{
namespace sym::codim_ipc_contact
{
    // C1 clamping
    inline __device__ void f0(Float x2, Float epsvh, Float& f0)
    {
        if(x2 >= epsvh * epsvh)
        {
            //tex: $$y$$
            f0 = std::sqrt(x2);
        }
        else
        {
            //tex: $$\frac{y^{2}}{\epsilon_{x}} + \frac{1}{3 \epsilon_{x}} - \frac{y^{3}}{3 \epsilon_{x}^{2}}$$
            f0 = x2 * (-std::sqrt(x2) / 3.0 + epsvh) / (epsvh * epsvh) + epsvh / 3.0;
        }
    }

    inline __device__ void f1_div_rel_dx_norm(Float x2, Float epsvh, Float& result)
    {
        if(x2 >= epsvh * epsvh)
        {
            //tex: $$ \frac{1}{y}$$
            result = 1 / std::sqrt(x2);
        }
        else
        {
            //tex: $$ \frac{2 \epsilon_{x} - y}{ \epsilon_{x}^{2}}$$
            result = (-std::sqrt(x2) + 2.0 * epsvh) / (epsvh * epsvh);
        }
    }

    inline __device__ void f2_term(Float x2, Float epsvh, Float& term)
    {
        term = -1 / (epsvh * epsvh);
        // same for x2 >= epsvh * epsvh for C1 clamped friction
    }


    inline __device__ Float friction_energy(Float mu, Float lambda, Float eps_vh, Vector2 tan_rel_x)
    {
        Float f0_val;
        f0(tan_rel_x.squaredNorm(), eps_vh, f0_val);
        return mu * lambda * f0_val;
    }

    inline __device__ void friction_gradient(Vector2& G2, Float mu, Float lambda, Float eps_vh, Vector2 tan_rel_x)
    {
        Float f1_val;
        f1_div_rel_dx_norm(tan_rel_x.squaredNorm(), eps_vh, f1_val);
        G2 = mu * lambda * f1_val * tan_rel_x;
    }

    inline __device__ void make_spd(Matrix2x2& mat)
    {
        Vector2   eigen_values;
        Matrix2x2 eigen_vectors;
        muda::eigen::template evd(mat, eigen_values, eigen_vectors);
#pragma unroll
        for(int i = 0; i < 2; ++i)
        {
            auto& v = eigen_values(i);
            v       = v < 0.0 ? 0.0 : v;
        }
        mat = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    };

    inline __device__ void friction_hessian(Matrix2x2& H2x2, Float mu, Float lambda, Float eps_vh, Vector2 tan_rel_x)
    {
        Float sq_norm = tan_rel_x.squaredNorm();
        Float epsvh2  = eps_vh * eps_vh;
        Float f1_div_rel_dx_norm_val;
        f1_div_rel_dx_norm(tan_rel_x.squaredNorm(), eps_vh, f1_div_rel_dx_norm_val);

        if(sq_norm >= epsvh2)
        {

            // no SPD projection needed
            Vector2 ubar(-tan_rel_x[1], tan_rel_x[0]);
            H2x2 = ubar * (mu * lambda * f1_div_rel_dx_norm_val / sq_norm)
                   * ubar.transpose();
        }
        else
        {
            if(sq_norm == 0.0)
            {
                // no SPD projection needed
                H2x2 = (mu * lambda * f1_div_rel_dx_norm_val) * Matrix2x2::Identity();
            }
            else
            {
                Float f2_term_val;
                f2_term(sq_norm, eps_vh, f2_term_val);

                Float relDXNorm = std::sqrt(sq_norm);

                // only need to project the inner 2x2 matrix to SPD
                H2x2 = tan_rel_x * (f2_term_val / relDXNorm) * tan_rel_x.transpose();
                H2x2.diagonal().array() += f1_div_rel_dx_norm_val;

                make_spd(H2x2);

                H2x2 *= mu * lambda;
            }
        }
    }

    inline __device__ Float normal_force(Float kappa, Float d_hat, Float thickness, Float D)
    {
        using namespace distance;
        Float d = std::sqrt(D);
        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, d_hat, thickness);
        Float dBdd = dBdD * 2.0 * d;
        return -dBdd;  // > 0
    }

    inline __device__ void PT_friction_basis(
        // out
        Float&               f,
        Vector2&             beta,
        Matrix<Float, 3, 2>& basis,
        Vector2&             tan_rel_dx,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_P,
        const Vector3& prev_T0,
        const Vector3& prev_T1,
        const Vector3& prev_T2,
        const Vector3& P,
        const Vector3& T0,
        const Vector3& T1,
        const Vector3& T2)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force

        point_triangle_closest_point(prev_P, prev_T0, prev_T1, prev_T2, beta);
        point_triangle_tangent_basis(prev_P, prev_T0, prev_T1, prev_T2, basis);
        Float prev_D;
        point_triangle_distance2(prev_P, prev_T0, prev_T1, prev_T2, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dP  = P - prev_P;
        Vector3 dT0 = T0 - prev_T0;
        Vector3 dT1 = T1 - prev_T1;
        Vector3 dT2 = T2 - prev_T2;

        point_triangle_tan_rel_dx(dP, dT0, dT1, dT2, basis, beta, tan_rel_dx);
    }

    inline __device__ Float PT_friction_energy(Float          kappa,
                                               Float          d_hat,
                                               Float          thickness,
                                               Float          mu,
                                               Float          eps_vh,
                                               const Vector3& prev_P,
                                               const Vector3& prev_T0,
                                               const Vector3& prev_T1,
                                               const Vector3& prev_T2,
                                               const Vector3& P,
                                               const Vector3& T0,
                                               const Vector3& T1,
                                               const Vector3& T2)
    {
        using namespace distance;
        using namespace friction;


        Float               f;
        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        PT_friction_basis(
            // out
            f,
            beta,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_P,
            prev_T0,
            prev_T1,
            prev_T2,
            P,
            T0,
            T1,
            T2);

        Float E = friction_energy(mu, f, eps_vh, tan_rel_dx);

        return E;
    }

    inline __device__ void PT_friction_gradient_hessian(Vector12&    G,
                                                        Matrix12x12& H,
                                                        Float        kappa,
                                                        Float        d_hat,
                                                        Float        thickness,
                                                        Float        mu,
                                                        Float        eps_vh,
                                                        const Vector3& prev_P,
                                                        const Vector3& prev_T0,
                                                        const Vector3& prev_T1,
                                                        const Vector3& prev_T2,
                                                        const Vector3& P,
                                                        const Vector3& T0,
                                                        const Vector3& T1,
                                                        const Vector3& T2)
    {
        using namespace distance;
        using namespace friction;

        Float               f;
        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        PT_friction_basis(
            // out
            f,
            beta,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_P,
            prev_T0,
            prev_T1,
            prev_T2,
            P,
            T0,
            T1,
            T2);

        Matrix<Float, 2, 12> J;
        point_triangle_jacobi(basis, beta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        //apply_point_triangle_jacobi(G2, basis, beta, G);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        H = J.transpose() * H2x2 * J;
    }


    inline __device__ void EE_friction_basis(
        // out
        Float&               f,
        Vector2&             gamma,
        Matrix<Float, 3, 2>& basis,
        Vector2&             tan_rel_dx,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_Ea0,
        const Vector3& prev_Ea1,
        const Vector3& prev_Eb0,
        const Vector3& prev_Eb1,
        const Vector3& Ea0,
        const Vector3& Ea1,
        const Vector3& Eb0,
        const Vector3& Eb1)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force

        edge_edge_closest_point(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, gamma);
        edge_edge_tangent_basis(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, basis);

        Float prev_D;
        edge_edge_distance2(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dEa0 = Ea0 - prev_Ea0;
        Vector3 dEa1 = Ea1 - prev_Ea1;
        Vector3 dEb0 = Eb0 - prev_Eb0;
        Vector3 dEb1 = Eb1 - prev_Eb1;

        edge_edge_tan_rel_dx(dEa0, dEa1, dEb0, dEb1, basis, gamma, tan_rel_dx);
    }

    inline __device__ Float EE_friction_energy(Float          kappa,
                                               Float          d_hat,
                                               Float          thickness,
                                               Float          mu,
                                               Float          eps_vh,
                                               const Vector3& prev_Ea0,
                                               const Vector3& prev_Ea1,
                                               const Vector3& prev_Eb0,
                                               const Vector3& prev_Eb1,
                                               const Vector3& Ea0,
                                               const Vector3& Ea1,
                                               const Vector3& Eb0,
                                               const Vector3& Eb1)
    {
        Float               f;
        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        EE_friction_basis(
            // out
            f,
            gamma,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_Ea0,
            prev_Ea1,
            prev_Eb0,
            prev_Eb1,
            Ea0,
            Ea1,
            Eb0,
            Eb1);

        Float E = friction_energy(mu, f, eps_vh, tan_rel_dx);

        return E;
    }

    inline __device__ void EE_friction_gradient_hessian(Vector12&    G,
                                                        Matrix12x12& H,
                                                        Float        kappa,
                                                        Float        d_hat,
                                                        Float        thickness,
                                                        Float        mu,
                                                        Float        eps_vh,
                                                        const Vector3& prev_Ea0,
                                                        const Vector3& prev_Ea1,
                                                        const Vector3& prev_Eb0,
                                                        const Vector3& prev_Eb1,
                                                        const Vector3& Ea0,
                                                        const Vector3& Ea1,
                                                        const Vector3& Eb0,
                                                        const Vector3& Eb1)
    {
        using namespace friction;

        Float               f;
        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        EE_friction_basis(
            // out
            f,
            gamma,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_Ea0,
            prev_Ea1,
            prev_Eb0,
            prev_Eb1,
            Ea0,
            Ea1,
            Eb0,
            Eb1);

        Matrix<Float, 2, 12> J;
        edge_edge_jacobi(basis, gamma, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        //apply_edge_edge_jacobi(G2, basis, gamma, G);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        H = J.transpose() * H2x2 * J;
    }

    inline __device__ Float PE_friction_basis(
        // out
        Float&               f,
        Float&               eta,
        Matrix<Float, 3, 2>& basis,
        Vector2&             tan_rel_dx,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_P,
        const Vector3& prev_E0,
        const Vector3& prev_E1,
        const Vector3& P,
        const Vector3& E0,
        const Vector3& E1)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force

        point_edge_closest_point(prev_P, prev_E0, prev_E1, eta);
        point_edge_tangent_basis(prev_P, prev_E0, prev_E1, basis);

        Float prev_D;
        point_edge_distance2(prev_P, prev_E0, prev_E1, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dP  = P - prev_P;
        Vector3 dE0 = E0 - prev_E0;
        Vector3 dE1 = E1 - prev_E1;

        point_edge_tan_rel_dx(dP, dE0, dE1, basis, eta, tan_rel_dx);
    }


    inline __device__ Float PE_friction_energy(Float          kappa,
                                               Float          d_hat,
                                               Float          thickness,
                                               Float          mu,
                                               Float          eps_vh,
                                               const Vector3& prev_P,
                                               const Vector3& prev_E0,
                                               const Vector3& prev_E1,
                                               const Vector3& P,
                                               const Vector3& E0,
                                               const Vector3& E1)
    {
        using namespace friction;

        Float               f;
        Float               eta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        PE_friction_basis(
            // out
            f,
            eta,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_P,
            prev_E0,
            prev_E1,
            P,
            E0,
            E1);

        Float E = friction_energy(mu, f, eps_vh, tan_rel_dx);

        return E;
    }

    inline __device__ void PE_friction_gradient_hessian(Vector9&   G,
                                                        Matrix9x9& H,
                                                        Float      kappa,
                                                        Float      d_hat,
                                                        Float      thickness,
                                                        Float      mu,
                                                        Float      eps_vh,
                                                        const Vector3& prev_P,
                                                        const Vector3& prev_E0,
                                                        const Vector3& prev_E1,
                                                        const Vector3& P,
                                                        const Vector3& E0,
                                                        const Vector3& E1)
    {
        using namespace distance;
        using namespace friction;

        Float               f;
        Float               eta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        PE_friction_basis(
            // out
            f,
            eta,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_P,
            prev_E0,
            prev_E1,
            P,
            E0,
            E1);

        Matrix<Float, 2, 9> J;
        point_edge_jacobi(basis, eta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        //apply_point_edge_jacobi(G2, basis, eta, G);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        H = J.transpose() * H2x2 * J;
    }

    inline __device__ Float PP_friction_basis(
        // out
        Float&               f,
        Matrix<Float, 3, 2>& basis,
        Vector2&             tan_rel_dx,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_P0,
        const Vector3& prev_P1,
        const Vector3& P0,
        const Vector3& P1)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force
        point_point_tangent_basis(prev_P0, prev_P1, basis);

        Float prev_D;
        point_point_distance2(prev_P0, prev_P1, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dP0 = P0 - prev_P0;
        Vector3 dP1 = P1 - prev_P1;

        point_point_tan_rel_dx(dP0, dP1, basis, tan_rel_dx);
    }


    inline __device__ Float PP_friction_energy(Float          kappa,
                                               Float          d_hat,
                                               Float          thickness,
                                               Float          mu,
                                               Float          eps_vh,
                                               const Vector3& prev_P0,
                                               const Vector3& prev_P1,
                                               const Vector3& P0,
                                               const Vector3& P1)
    {
        using namespace distance;
        using namespace friction;

        Float               f;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        PP_friction_basis(
            // out
            f,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_P0,
            prev_P1,
            P0,
            P1);

        Float E = friction_energy(mu, f, eps_vh, tan_rel_dx);

        return E;
    }

    inline __device__ void PP_friction_gradient_hessian(Vector6&   G,
                                                        Matrix6x6& H,
                                                        Float      kappa,
                                                        Float      d_hat,
                                                        Float      thickness,
                                                        Float      mu,
                                                        Float      eps_vh,
                                                        const Vector3& prev_P0,
                                                        const Vector3& prev_P1,
                                                        const Vector3& P0,
                                                        const Vector3& P1)
    {
        using namespace distance;
        using namespace friction;

        Float               f;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        PP_friction_basis(
            // out
            f,
            basis,
            tan_rel_dx,
            // in
            kappa,
            d_hat,
            thickness,
            prev_P0,
            prev_P1,
            P0,
            P1);

        Matrix<Float, 2, 6> J;
        point_point_jacobi(basis, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        //apply_point_point_jacobi(G2, basis, G);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        H = J.transpose() * H2x2 * J;
    }

}  // namespace sym::codim_ipc_contact
}  // namespace uipc::backend::cuda
