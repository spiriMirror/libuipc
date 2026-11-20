/*
 * NVIDIA contributions: lift_friction_hessian and templated Hessian outputs
 * for the PT/EE/PE/PP friction derivative routines, and the split of the
 * friction basis into frame-lagged quantities and the tangential relative
 * displacement.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <type_define.h>
#include <contact_system/contact_coeff.h>
#include <contact_system/contact_models/codim_ipc_contact_function.h>

namespace uipc::backend::cuda
{
namespace sym::codim_ipc_contact
{

    inline __device__ ContactCoeff PT_contact_coeff(const cuda_tool::CDense2D<ContactCoeff>& table,
                                                    const Vector4i& cids)
    {
        Float kappa = 0.0;
        Float mu    = 0.0;
        for(int j = 1; j < 4; ++j)
        {
            ContactCoeff coeff = table(cids[0], cids[j]);
            kappa += coeff.kappa;
            mu += coeff.mu;
        }
        return {kappa / 3.0, mu / 3.0};
    }

    inline __device__ ContactCoeff EE_contact_coeff(const cuda_tool::CDense2D<ContactCoeff>& table,
                                                    const Vector4i& cids)
    {
        Float kappa = 0.0;
        Float mu    = 0.0;
        for(int j = 0; j < 2; ++j)
        {
            for(int k = 2; k < 4; ++k)
            {
                ContactCoeff coeff = table(cids[j], cids[k]);
                kappa += coeff.kappa;
                mu += coeff.mu;
            }
        }
        return {kappa / 4.0, mu / 4.0};
    }

    inline __device__ ContactCoeff PE_contact_coeff(const cuda_tool::CDense2D<ContactCoeff>& table,
                                                    const Vector3i& cids)
    {
        Float kappa = 0.0;
        Float mu    = 0.0;
        for(int j = 1; j < 3; ++j)
        {
            ContactCoeff coeff = table(cids[0], cids[j]);
            kappa += coeff.kappa;
            mu += coeff.mu;
        }
        return {kappa / 2.0, mu / 2.0};
    }

    inline __device__ ContactCoeff PP_contact_coeff(const cuda_tool::CDense2D<ContactCoeff>& table,
                                                    const Vector2i& cids)
    {
        return table(cids[0], cids[1]);
    }

    // friction_hessian() projects the only nonlinear part of the friction
    // Hessian, a 2x2 tangent-space matrix, before it reaches this function.
    // A congruence transform J^T H J preserves positive semidefiniteness when
    // the physical scale mu * normal_force is finite and nonnegative.  Return
    // whether callers may write the lifted Hessian directly.  Invalid scales
    // deliberately return false so the assembly kernel keeps its old full-size
    // projection as a fail-safe.
    template <typename Hessian, int N>
    inline __device__ bool lift_friction_hessian(Hessian&                   H,
                                                 const Matrix<Float, 2, N>& J,
                                                 const Matrix2x2& H2x2,
                                                 Float            signed_scale)
    {
        H = J.transpose() * H2x2 * J;
        return isfinite(signed_scale) && signed_scale >= 0.0;
    }

    // The lagged part of the PT friction basis: closest-point weights,
    // tangent basis and normal force at the frame-start positions.
    inline __device__ void PT_friction_lagged(
        // out
        Float&               f,
        Vector2&             beta,
        Matrix<Float, 3, 2>& basis,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_P,
        const Vector3& prev_T0,
        const Vector3& prev_T1,
        const Vector3& prev_T2)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force

        point_triangle_closest_point(prev_P, prev_T0, prev_T1, prev_T2, beta);
        point_triangle_tangent_basis(prev_P, prev_T0, prev_T1, prev_T2, basis);
        Float prev_D;
        point_triangle_distance2(prev_P, prev_T0, prev_T1, prev_T2, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);
    }

    inline __device__ void PT_friction_tan_rel_dx(Vector2& tan_rel_dx,
                                                  const Matrix<Float, 3, 2>& basis,
                                                  const Vector2& beta,
                                                  const Vector3& prev_P,
                                                  const Vector3& prev_T0,
                                                  const Vector3& prev_T1,
                                                  const Vector3& prev_T2,
                                                  const Vector3& P,
                                                  const Vector3& T0,
                                                  const Vector3& T1,
                                                  const Vector3& T2)
    {
        using namespace friction;

        Vector3 dP  = P - prev_P;
        Vector3 dT0 = T0 - prev_T0;
        Vector3 dT1 = T1 - prev_T1;
        Vector3 dT2 = T2 - prev_T2;

        point_triangle_tan_rel_dx(dP, dT0, dT1, dT2, basis, beta, tan_rel_dx);
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
        PT_friction_lagged(f, beta, basis, kappa, d_hat, thickness, prev_P, prev_T0, prev_T1, prev_T2);
        PT_friction_tan_rel_dx(
            tan_rel_dx, basis, beta, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
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

    template <typename Hessian>
    inline __device__ bool PT_friction_gradient_hessian(Vector12& G,
                                                        Hessian&  H,
                                                        Float     kappa,
                                                        Float     d_hat,
                                                        Float     thickness,
                                                        Float     mu,
                                                        Float     eps_vh,
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
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void PT_friction_gradient(Vector12&      G,
                                                Float          kappa,
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
        G = J.transpose() * G2;
    }


    // The lagged part of the EE friction basis (see PT_friction_lagged).
    inline __device__ void EE_friction_lagged(
        // out
        Float&               f,
        Vector2&             gamma,
        Matrix<Float, 3, 2>& basis,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_Ea0,
        const Vector3& prev_Ea1,
        const Vector3& prev_Eb0,
        const Vector3& prev_Eb1)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force

        edge_edge_closest_point(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, gamma);
        edge_edge_tangent_basis(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, basis);

        Float prev_D;
        edge_edge_distance2(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);
    }

    inline __device__ void EE_friction_tan_rel_dx(Vector2& tan_rel_dx,
                                                  const Matrix<Float, 3, 2>& basis,
                                                  const Vector2& gamma,
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

        Vector3 dEa0 = Ea0 - prev_Ea0;
        Vector3 dEa1 = Ea1 - prev_Ea1;
        Vector3 dEb0 = Eb0 - prev_Eb0;
        Vector3 dEb1 = Eb1 - prev_Eb1;

        edge_edge_tan_rel_dx(dEa0, dEa1, dEb0, dEb1, basis, gamma, tan_rel_dx);
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
        EE_friction_lagged(f, gamma, basis, kappa, d_hat, thickness, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1);
        EE_friction_tan_rel_dx(
            tan_rel_dx, basis, gamma, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
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

    template <typename Hessian>
    inline __device__ bool EE_friction_gradient_hessian(Vector12& G,
                                                        Hessian&  H,
                                                        Float     kappa,
                                                        Float     d_hat,
                                                        Float     thickness,
                                                        Float     mu,
                                                        Float     eps_vh,
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
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void EE_friction_gradient(Vector12&      G,
                                                Float          kappa,
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
        G = J.transpose() * G2;
    }

    // The lagged part of the PE friction basis (see PT_friction_lagged).
    inline __device__ void PE_friction_lagged(
        // out
        Float&               f,
        Float&               eta,
        Matrix<Float, 3, 2>& basis,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_P,
        const Vector3& prev_E0,
        const Vector3& prev_E1)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force

        point_edge_closest_point(prev_P, prev_E0, prev_E1, eta);
        point_edge_tangent_basis(prev_P, prev_E0, prev_E1, basis);

        Float prev_D;
        point_edge_distance2(prev_P, prev_E0, prev_E1, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);
    }

    inline __device__ void PE_friction_tan_rel_dx(Vector2& tan_rel_dx,
                                                  const Matrix<Float, 3, 2>& basis,
                                                  Float          eta,
                                                  const Vector3& prev_P,
                                                  const Vector3& prev_E0,
                                                  const Vector3& prev_E1,
                                                  const Vector3& P,
                                                  const Vector3& E0,
                                                  const Vector3& E1)
    {
        using namespace friction;

        Vector3 dP  = P - prev_P;
        Vector3 dE0 = E0 - prev_E0;
        Vector3 dE1 = E1 - prev_E1;

        point_edge_tan_rel_dx(dP, dE0, dE1, basis, eta, tan_rel_dx);
    }

    inline __device__ void PE_friction_basis(
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
        PE_friction_lagged(f, eta, basis, kappa, d_hat, thickness, prev_P, prev_E0, prev_E1);
        PE_friction_tan_rel_dx(tan_rel_dx, basis, eta, prev_P, prev_E0, prev_E1, P, E0, E1);
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

    template <typename Hessian>
    inline __device__ bool PE_friction_gradient_hessian(Vector9& G,
                                                        Hessian& H,
                                                        Float    kappa,
                                                        Float    d_hat,
                                                        Float    thickness,
                                                        Float    mu,
                                                        Float    eps_vh,
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
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void PE_friction_gradient(Vector9&       G,
                                                Float          kappa,
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

        Matrix<Float, 2, 9> J;
        point_edge_jacobi(basis, eta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;
    }

    // The lagged part of the PP friction basis (see PT_friction_lagged).
    inline __device__ void PP_friction_lagged(
        // out
        Float&               f,
        Matrix<Float, 3, 2>& basis,
        // in
        Float          kappa,
        Float          d_hat,
        Float          thickness,
        const Vector3& prev_P0,
        const Vector3& prev_P1)
    {
        using namespace distance;
        using namespace friction;

        // using the prev values to compute normal force
        point_point_tangent_basis(prev_P0, prev_P1, basis);

        Float prev_D;
        point_point_distance2(prev_P0, prev_P1, prev_D);

        f = normal_force(kappa, d_hat, thickness, prev_D);
    }

    inline __device__ void PP_friction_tan_rel_dx(Vector2& tan_rel_dx,
                                                  const Matrix<Float, 3, 2>& basis,
                                                  const Vector3& prev_P0,
                                                  const Vector3& prev_P1,
                                                  const Vector3& P0,
                                                  const Vector3& P1)
    {
        using namespace friction;

        Vector3 dP0 = P0 - prev_P0;
        Vector3 dP1 = P1 - prev_P1;

        point_point_tan_rel_dx(dP0, dP1, basis, tan_rel_dx);
    }

    inline __device__ void PP_friction_basis(
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
        PP_friction_lagged(f, basis, kappa, d_hat, thickness, prev_P0, prev_P1);
        PP_friction_tan_rel_dx(tan_rel_dx, basis, prev_P0, prev_P1, P0, P1);
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

    template <typename Hessian>
    inline __device__ bool PP_friction_gradient_hessian(Vector6& G,
                                                        Hessian& H,
                                                        Float    kappa,
                                                        Float    d_hat,
                                                        Float    thickness,
                                                        Float    mu,
                                                        Float    eps_vh,
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
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void PP_friction_gradient(Vector6&       G,
                                                Float          kappa,
                                                Float          d_hat,
                                                Float          thickness,
                                                Float          mu,
                                                Float          eps_vh,
                                                const Vector3& prev_P0,
                                                const Vector3& prev_P1,
                                                const Vector3& P0,
                                                const Vector3& P1)
    {
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
        G = J.transpose() * G2;
    }

    // Lagged friction quantities of one contact pair, computed once per
    // frame from the frame-start positions with the *_friction_lagged
    // functions above: [0] the normal force, [1..2] the closest-point
    // weights (PT beta, EE gamma, PE eta in [1]), [3..8] the 3x2 tangent
    // basis (column-major) and [9] 1 for an EE pair whose friction is
    // skipped by the mollifier. The overloads below that take it evaluate
    // exactly the operations of the recomputing ones on the current
    // positions only.
    using FrictionLagged = Vector<Float, 10>;

    inline __device__ void pack_friction_lagged(FrictionLagged& L,
                                                Float           f,
                                                const Vector2&  w,
                                                const Matrix<Float, 3, 2>& basis,
                                                bool mollified)
    {
        L[0] = f;
        L[1] = w[0];
        L[2] = w[1];
        for(int j = 0; j < 2; ++j)
            for(int i = 0; i < 3; ++i)
                L[3 + 3 * j + i] = basis(i, j);
        L[9] = mollified ? 1.0 : 0.0;
    }

    inline __device__ void unpack_friction_lagged(const FrictionLagged& L,
                                                  Float&                f,
                                                  Vector2&              w,
                                                  Matrix<Float, 3, 2>&  basis)
    {
        f    = L[0];
        w[0] = L[1];
        w[1] = L[2];
        for(int j = 0; j < 2; ++j)
            for(int i = 0; i < 3; ++i)
                basis(i, j) = L[3 + 3 * j + i];
    }

    inline __device__ bool friction_lagged_mollified(const FrictionLagged& L)
    {
        return L[9] != 0.0;
    }

    inline __device__ Float PT_friction_energy(const FrictionLagged& L,
                                               Float                 mu,
                                               Float                 eps_vh,
                                               const Vector3&        prev_P,
                                               const Vector3&        prev_T0,
                                               const Vector3&        prev_T1,
                                               const Vector3&        prev_T2,
                                               const Vector3&        P,
                                               const Vector3&        T0,
                                               const Vector3&        T1,
                                               const Vector3&        T2)
    {
        Float               f;
        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, beta, basis);
        PT_friction_tan_rel_dx(
            tan_rel_dx, basis, beta, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
        return friction_energy(mu, f, eps_vh, tan_rel_dx);
    }

    template <typename Hessian>
    inline __device__ bool PT_friction_gradient_hessian(Vector12&             G,
                                                        Hessian&              H,
                                                        const FrictionLagged& L,
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
        using namespace friction;

        Float               f;
        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, beta, basis);
        PT_friction_tan_rel_dx(
            tan_rel_dx, basis, beta, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);

        Matrix<Float, 2, 12> J;
        point_triangle_jacobi(basis, beta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void PT_friction_gradient(Vector12&             G,
                                                const FrictionLagged& L,
                                                Float                 mu,
                                                Float                 eps_vh,
                                                const Vector3&        prev_P,
                                                const Vector3&        prev_T0,
                                                const Vector3&        prev_T1,
                                                const Vector3&        prev_T2,
                                                const Vector3&        P,
                                                const Vector3&        T0,
                                                const Vector3&        T1,
                                                const Vector3&        T2)
    {
        using namespace friction;

        Float               f;
        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, beta, basis);
        PT_friction_tan_rel_dx(
            tan_rel_dx, basis, beta, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);

        Matrix<Float, 2, 12> J;
        point_triangle_jacobi(basis, beta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;
    }

    inline __device__ Float EE_friction_energy(const FrictionLagged& L,
                                               Float                 mu,
                                               Float                 eps_vh,
                                               const Vector3&        prev_Ea0,
                                               const Vector3&        prev_Ea1,
                                               const Vector3&        prev_Eb0,
                                               const Vector3&        prev_Eb1,
                                               const Vector3&        Ea0,
                                               const Vector3&        Ea1,
                                               const Vector3&        Eb0,
                                               const Vector3&        Eb1)
    {
        Float               f;
        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, gamma, basis);
        EE_friction_tan_rel_dx(
            tan_rel_dx, basis, gamma, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
        return friction_energy(mu, f, eps_vh, tan_rel_dx);
    }

    template <typename Hessian>
    inline __device__ bool EE_friction_gradient_hessian(Vector12&             G,
                                                        Hessian&              H,
                                                        const FrictionLagged& L,
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
        using namespace friction;

        Float               f;
        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, gamma, basis);
        EE_friction_tan_rel_dx(
            tan_rel_dx, basis, gamma, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);

        Matrix<Float, 2, 12> J;
        edge_edge_jacobi(basis, gamma, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void EE_friction_gradient(Vector12&             G,
                                                const FrictionLagged& L,
                                                Float                 mu,
                                                Float                 eps_vh,
                                                const Vector3&        prev_Ea0,
                                                const Vector3&        prev_Ea1,
                                                const Vector3&        prev_Eb0,
                                                const Vector3&        prev_Eb1,
                                                const Vector3&        Ea0,
                                                const Vector3&        Ea1,
                                                const Vector3&        Eb0,
                                                const Vector3&        Eb1)
    {
        using namespace friction;

        Float               f;
        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, gamma, basis);
        EE_friction_tan_rel_dx(
            tan_rel_dx, basis, gamma, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);

        Matrix<Float, 2, 12> J;
        edge_edge_jacobi(basis, gamma, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;
    }

    inline __device__ Float PE_friction_energy(const FrictionLagged& L,
                                               Float                 mu,
                                               Float                 eps_vh,
                                               const Vector3&        prev_P,
                                               const Vector3&        prev_E0,
                                               const Vector3&        prev_E1,
                                               const Vector3&        P,
                                               const Vector3&        E0,
                                               const Vector3&        E1)
    {
        Float               f;
        Vector2             w;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, w, basis);
        PE_friction_tan_rel_dx(tan_rel_dx, basis, w[0], prev_P, prev_E0, prev_E1, P, E0, E1);
        return friction_energy(mu, f, eps_vh, tan_rel_dx);
    }

    template <typename Hessian>
    inline __device__ bool PE_friction_gradient_hessian(Vector9&              G,
                                                        Hessian&              H,
                                                        const FrictionLagged& L,
                                                        Float          mu,
                                                        Float          eps_vh,
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
        Vector2             w;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, w, basis);
        Float eta = w[0];
        PE_friction_tan_rel_dx(tan_rel_dx, basis, eta, prev_P, prev_E0, prev_E1, P, E0, E1);

        Matrix<Float, 2, 9> J;
        point_edge_jacobi(basis, eta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void PE_friction_gradient(Vector9&              G,
                                                const FrictionLagged& L,
                                                Float                 mu,
                                                Float                 eps_vh,
                                                const Vector3&        prev_P,
                                                const Vector3&        prev_E0,
                                                const Vector3&        prev_E1,
                                                const Vector3&        P,
                                                const Vector3&        E0,
                                                const Vector3&        E1)
    {
        using namespace friction;

        Float               f;
        Vector2             w;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, w, basis);
        Float eta = w[0];
        PE_friction_tan_rel_dx(tan_rel_dx, basis, eta, prev_P, prev_E0, prev_E1, P, E0, E1);

        Matrix<Float, 2, 9> J;
        point_edge_jacobi(basis, eta, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;
    }

    inline __device__ Float PP_friction_energy(const FrictionLagged& L,
                                               Float                 mu,
                                               Float                 eps_vh,
                                               const Vector3&        prev_P0,
                                               const Vector3&        prev_P1,
                                               const Vector3&        P0,
                                               const Vector3&        P1)
    {
        Float               f;
        Vector2             w;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, w, basis);
        PP_friction_tan_rel_dx(tan_rel_dx, basis, prev_P0, prev_P1, P0, P1);
        return friction_energy(mu, f, eps_vh, tan_rel_dx);
    }

    template <typename Hessian>
    inline __device__ bool PP_friction_gradient_hessian(Vector6&              G,
                                                        Hessian&              H,
                                                        const FrictionLagged& L,
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
        Vector2             w;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, w, basis);
        PP_friction_tan_rel_dx(tan_rel_dx, basis, prev_P0, prev_P1, P0, P1);

        Matrix<Float, 2, 6> J;
        point_point_jacobi(basis, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_rel_dx);
        return lift_friction_hessian(H, J, H2x2, mu * f);
    }

    inline __device__ void PP_friction_gradient(Vector6&              G,
                                                const FrictionLagged& L,
                                                Float                 mu,
                                                Float                 eps_vh,
                                                const Vector3&        prev_P0,
                                                const Vector3&        prev_P1,
                                                const Vector3&        P0,
                                                const Vector3&        P1)
    {
        using namespace friction;

        Float               f;
        Vector2             w;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;
        unpack_friction_lagged(L, f, w, basis);
        PP_friction_tan_rel_dx(tan_rel_dx, basis, prev_P0, prev_P1, P0, P1);

        Matrix<Float, 2, 6> J;
        point_point_jacobi(basis, J);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;
    }
}  // namespace sym::codim_ipc_contact
}  // namespace uipc::backend::cuda
