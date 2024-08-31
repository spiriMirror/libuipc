#pragma once
#include <type_define.h>
#include <utils/distance.h>
#include <contact_system/contact_models/ipc_contact_function.h>

namespace uipc::backend::cuda
{
namespace sym::ipc_contact
{
#include "sym/ipc_friction.inl"

    __device__ Float PT_friction_energy(Float          kappa,
                                        Float          squared_d_hat,
                                        Float          mu,
                                        Float          dt,
                                        const Vector3& P,
                                        const Vector3& T0,
                                        const Vector3& T1,
                                        const Vector3& T2,
                                        const Vector3& prev_P,
                                        const Vector3& prev_T0,
                                        const Vector3& prev_T1,
                                        const Vector3& prev_T2,
                                        Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        point_triangle_distance2(prev_P, prev_T0, prev_T1, prev_T2, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        point_triangle_distance2_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float   lam    = -dBdD * GradD.head(3).norm();
        Vector3 n      = (prev_T0 - prev_T1).cross(prev_T0 - prev_T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;
        Vector3 v1                   = (P - prev_P) / dt;

        // suppose P0 = t0 * T0 + t1 * T1 + t2 * T2
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0)         = prev_T1 - prev_T0;
        base.block<3, 1>(0, 1)         = prev_T2 - prev_T0;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base;
        Vector2                    rhs = base.transpose() * (prev_P - prev_T0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det     = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t     = Lhs_inv * rhs;
        Float   t1    = t(0);
        Float   t2    = t(1);
        Float   t0    = 1 - t1 - t2;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt
                          + t2 * (T2 - prev_T2) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float   y  = vk.norm() * dt;
        Float   F;
        FrictionEnergy(F, lam * mu, eps_v, dt, y);
        return F;
    }

    __device__ void PT_friction_gradient_hessian(Vector12&      G,
                                                 Matrix12x12&   H,
                                                 Float          kappa,
                                                 Float          squared_d_hat,
                                                 Float          mu,
                                                 Float          dt,
                                                 const Vector3& P,
                                                 const Vector3& T0,
                                                 const Vector3& T1,
                                                 const Vector3& T2,
                                                 const Vector3& prev_P,
                                                 const Vector3& prev_T0,
                                                 const Vector3& prev_T1,
                                                 const Vector3& prev_T2,
                                                 Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        point_triangle_distance2(prev_P, prev_T0, prev_T1, prev_T2, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        point_triangle_distance2_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float   lam    = -dBdD * GradD.head(3).norm();
        Vector3 n      = (prev_T0 - prev_T1).cross(prev_T0 - prev_T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;
        Vector3 v1                   = (P - prev_P) / dt;

        // suppose prev_P0 = t0 * prev_T0 + t1 * prev_T1 + t2 * prev_T2
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0)         = prev_T1 - prev_T0;
        base.block<3, 1>(0, 1)         = prev_T2 - prev_T0;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base;
        Vector2                    rhs = base.transpose() * (prev_P - prev_T0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det     = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t     = Lhs_inv * rhs;
        Float   t1    = t(0);
        Float   t2    = t(1);
        Float   t0    = 1 - t1 - t2;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt
                          + t2 * (T2 - prev_T2) / dt;
        Vector3                 vk = Tk.transpose() * V;
        Float                   y  = vk.norm() * dt;
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk, eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 12> GradV = Eigen::Matrix<Float, 6, 12>::Zero();
        GradV.block<3, 3>(0, 0)           = I / dt;
        GradV.block<3, 3>(3, 3)           = I * t0 / dt;
        GradV.block<3, 3>(3, 6)           = I * t1 / dt;
        GradV.block<3, 3>(3, 9)           = I * t2 / dt;
        Eigen::Matrix<Float, 12, 6> GradV_transpose = GradV.transpose();
        G                                           = GradV_transpose * dFdV;

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        H = GradV_transpose * ddFddV * GradV;
    }

    __device__ Float EE_friction_energy(Float          kappa,
                                        Float          squared_d_hat,
                                        Float          mu,
                                        Float          dt,
                                        const Vector3& P0,
                                        const Vector3& P1,
                                        const Vector3& Q0,
                                        const Vector3& Q1,
                                        const Vector3& prev_P0,
                                        const Vector3& prev_P1,
                                        const Vector3& prev_Q0,
                                        const Vector3& prev_Q1,
                                        Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        edge_edge_distance2(prev_P0, prev_P1, prev_Q0, prev_Q1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        edge_edge_distance2_gradient(prev_P0, prev_P1, prev_Q0, prev_Q1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Vector3 n      = (prev_P0 - prev_P1).cross(prev_Q0 - prev_Q1);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;

        // suppose prev_P_project = prev_P0 + t0 * (prev_P1 - prev_P0)
        //         prev_Q_project = prev_Q0 + t1 * (prev_Q1 - prev_Q0)
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0)         = prev_P1 - prev_P0;
        base.block<3, 1>(0, 1)         = prev_Q1 - prev_Q0;
        Eigen::Matrix<Float, 2, 2> X   = Eigen::Matrix<Float, 2, 2>::Zero();
        X(0, 0)                        = -1;
        X(1, 1)                        = 1;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base * X;
        Vector2                    rhs = base.transpose() * (prev_P0 - prev_Q0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det     = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t     = Lhs_inv * rhs;
        Float   t0    = t(0);
        Float   t1    = t(1);

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P0 - prev_P0) * (1 - t0) / dt + (P1 - prev_P1) * t0 / dt;
        V.segment<3>(3) = (Q0 - prev_Q0) * (1 - t1) / dt + (Q1 - prev_Q1) * t1 / dt;
        Vector3 vk = Tk.transpose() * V;
        Float   y  = vk.norm() * dt;
        Float   lam =
            -dBdD * (GradD.segment<3>(0) * (1 - t0) + GradD.segment<3>(3) * t0).norm();
        Float F;
        FrictionEnergy(F, lam * mu, eps_v, dt, y);
        return F;
    }

    __device__ void EE_friction_gradient_hessian(Vector12&      G,
                                                 Matrix12x12&   H,
                                                 Float          kappa,
                                                 Float          squared_d_hat,
                                                 Float          mu,
                                                 Float          dt,
                                                 const Vector3& P0,
                                                 const Vector3& P1,
                                                 const Vector3& Q0,
                                                 const Vector3& Q1,
                                                 const Vector3& prev_P0,
                                                 const Vector3& prev_P1,
                                                 const Vector3& prev_Q0,
                                                 const Vector3& prev_Q1,
                                                 Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        edge_edge_distance2(prev_P0, prev_P1, prev_Q0, prev_Q1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        edge_edge_distance2_gradient(prev_P0, prev_P1, prev_Q0, prev_Q1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Vector3 n      = (prev_P0 - prev_P1).cross(prev_Q0 - prev_Q1);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;

        // suppose prev_P_project = prev_P0 + t0 * (prev_P1 - prev_P0)
        //         prev_Q_project = prev_Q0 + t1 * (prev_Q1 - prev_Q0)
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0)         = prev_P1 - prev_P0;
        base.block<3, 1>(0, 1)         = prev_Q1 - prev_Q0;
        Eigen::Matrix<Float, 2, 2> X   = Eigen::Matrix<Float, 2, 2>::Zero();
        X(0, 0)                        = -1;
        X(1, 1)                        = 1;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base * X;
        Vector2                    rhs = base.transpose() * (prev_P0 - prev_Q0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det     = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t     = Lhs_inv * rhs;
        Float   t0    = t(0);
        Float   t1    = t(1);

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P0 - prev_P0) * (1 - t0) / dt + (P1 - prev_P1) * t0 / dt;
        V.segment<3>(3) = (Q0 - prev_Q0) * (1 - t1) / dt + (Q1 - prev_Q1) * t1 / dt;
        Vector3 vk = Tk.transpose() * V;
        Float   y  = vk.norm() * dt;
        Float   lam =
            -dBdD * (GradD.segment<3>(0) * (1 - t0) + GradD.segment<3>(3) * t0).norm();

        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk, eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 12> GradV = Eigen::Matrix<Float, 6, 12>::Zero();
        GradV.block<3, 3>(0, 0)           = I * (1 - t0) / dt;
        GradV.block<3, 3>(0, 3)           = I * t0 / dt;
        GradV.block<3, 3>(3, 6)           = I * (1 - t1) / dt;
        GradV.block<3, 3>(3, 9)           = I * t1 / dt;
        Eigen::Matrix<Float, 12, 6> GradV_transpose = GradV.transpose();
        G                                           = GradV_transpose * dFdV;

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        H = GradV_transpose * ddFddV * GradV;
    }

    __device__ Float PE_friction_energy(Float          kappa,
                                        Float          squared_d_hat,
                                        Float          mu,
                                        Float          dt,
                                        const Vector3& P,
                                        const Vector3& E0,
                                        const Vector3& E1,
                                        const Vector3& prev_P,
                                        const Vector3& prev_E0,
                                        const Vector3& prev_E1,
                                        Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        point_edge_distance2(prev_P, prev_E0, prev_E1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector9 GradD;
        point_edge_distance2_gradient(prev_P, prev_E0, prev_E1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();
        // suppose prev_P0 = t0 * prev_E0 + t1 * prev_E1
        Float t0 = (prev_P - prev_E1).dot(prev_E0 - prev_E1)
                   / (prev_E0 - prev_E1).squaredNorm();
        Float t1 = 1 - t0;

        Vector3                    prev_P0 = t0 * prev_E0 + t1 * prev_E1;
        Vector3                    n       = prev_P0 - prev_P;
        Vector3                    normal  = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;
        Vector3 v1                   = (P - prev_P) / dt;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (E0 - prev_E0) / dt + t1 * (E1 - prev_E1) / dt;
        Vector3 vk      = Tk.transpose() * V;
        Float   y       = vk.norm() * dt;
        Float   F;
        FrictionEnergy(F, lam * mu, eps_v, dt, y);
        return F;
    }

    __device__ void PE_friction_gradient_hessian(Vector9&       G,
                                                 Matrix9x9&     H,
                                                 Float          kappa,
                                                 Float          squared_d_hat,
                                                 Float          mu,
                                                 Float          dt,
                                                 const Vector3& P,
                                                 const Vector3& E0,
                                                 const Vector3& E1,
                                                 const Vector3& prev_P,
                                                 const Vector3& prev_E0,
                                                 const Vector3& prev_E1,
                                                 Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        point_edge_distance2(prev_P, prev_E0, prev_E1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector9 GradD = Vector9::Zero();
        point_edge_distance2_gradient(prev_P, prev_E0, prev_E1, GradD);

        Float dBdD = 0;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float lam = -dBdD * GradD.head(3).norm();

        // suppose prev_P0 = t0 * prev_E0 + t1 * prev_E1
        Float t0 = (prev_P - prev_E1).dot(prev_E0 - prev_E1)
                   / (prev_E0 - prev_E1).squaredNorm();
        Float t1 = 1 - t0;

        Vector3                    prev_P0 = t0 * prev_E0 + t1 * prev_E1;
        Vector3                    n       = prev_P0 - prev_P;
        Vector3                    normal  = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;
        Vector3 v1                   = (P - prev_P) / dt;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (E0 - prev_E0) / dt + t1 * (E1 - prev_E1) / dt;
        Vector3                 vk = Tk.transpose() * V;
        Float                   y  = vk.norm() * dt;
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk, eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 9> GradV = Eigen::Matrix<Float, 6, 9>::Zero();
        GradV.block<3, 3>(0, 0)          = I / dt;
        GradV.block<3, 3>(3, 3)          = I * t0 / dt;
        GradV.block<3, 3>(3, 6)          = I * t1 / dt;
        Eigen::Matrix<Float, 9, 6> GradV_transpose = GradV.transpose();
        G                                          = GradV_transpose * dFdV;

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        H = GradV_transpose * ddFddV * GradV;
    }

    __device__ Float PP_friction_energy(Float          kappa,
                                        Float          squared_d_hat,
                                        Float          mu,
                                        Float          dt,
                                        const Vector3& P,
                                        const Vector3& Q,
                                        const Vector3& prev_P,
                                        const Vector3& prev_Q,
                                        Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        point_point_distance2(prev_P, prev_Q, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector6 GradD;
        point_point_distance2_gradient(prev_P, prev_Q, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();

        Vector3                    n      = prev_Q - prev_P;
        Vector3                    normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P - prev_P) / dt;
        V.segment<3>(3) = (Q - prev_Q) / dt;
        Vector3 vk      = Tk.transpose() * V;
        Float   y       = vk.norm() * dt;
        Float   F;
        FrictionEnergy(F, lam * mu, eps_v, dt, y);
        return F;
    }

    __device__ void PP_friction_gradient_hessian(Vector6&       G,
                                                 Matrix6x6&     H,
                                                 Float          kappa,
                                                 Float          squared_d_hat,
                                                 Float          mu,
                                                 Float          dt,
                                                 const Vector3& P,
                                                 const Vector3& Q,
                                                 const Vector3& prev_P,
                                                 const Vector3& prev_Q,
                                                 Float          eps_v)
    {
        using namespace distance;
        using namespace sym::ipc_contact;

        Float D;
        point_point_distance2(prev_P, prev_Q, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector6 GradD;
        point_point_distance2_gradient(prev_P, prev_Q, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();

        Vector3                    n      = prev_Q - prev_P;
        Vector3                    normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0)         = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0)         = normal * normal.transpose() - I;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0)            = (P - prev_P) / dt;
        V.segment<3>(3)            = (Q - prev_Q) / dt;
        Vector3                 vk = Tk.transpose() * V;
        Float                   y  = vk.norm() * dt;
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk, eps_v, dt, vk);
        // GradV = Eigen::Matrix<Float, 6, 6>::Identity();
        G = dFdV;

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        H = ddFddV;
    }

}  // namespace sym::ipc_simplex_contact
}  // namespace uipc::backend::cuda
