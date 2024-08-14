#include <contact_system/contact_models/ipc_simplex_frictional_contact.h>
#include <contact_system/contact_models/ipc_simplex_contact_function.h>

#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexFrictionalContact);

void IPCSimplexFrictionalContact::do_build(BuildInfo& info) {}

namespace sym::ipc_simplex_contact
{
#include "sym/ipc_friction.inl"
#include "sym/ipc_contact.inl"

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
        using namespace muda::distance;
        Float D;
        point_triangle_distance(prev_P, prev_T0, prev_T1, prev_T2, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        point_triangle_distance_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();
        Vector3 n =  (prev_T0 - prev_T1).cross(prev_T0 - prev_T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        // suppose P0 = t0 * T0 + t1 * T1 + t2 * T2
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0) = prev_T1 - prev_T0;
        base.block<3, 1>(0, 1) = prev_T2 - prev_T0;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base;
        Vector2 rhs = base.transpose() * (prev_P - prev_T0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t = Lhs_inv * rhs;
        Float t1 = t(0);
        Float t2 = t(1);
        Float t0 = 1 - t1 - t2;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt + t2 * (T2 - prev_T2) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Float F;
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
        using namespace muda::distance;
        Float D;
        point_triangle_distance(prev_P, prev_T0, prev_T1, prev_T2, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        point_triangle_distance_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float lam = -dBdD * GradD.head(3).norm();
        Vector3 n =  (prev_T0 - prev_T1).cross(prev_T0 - prev_T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        // suppose prev_P0 = t0 * prev_T0 + t1 * prev_T1 + t2 * prev_T2
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0) = prev_T1 - prev_T0;
        base.block<3, 1>(0, 1) = prev_T2 - prev_T0;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base;
        Vector2 rhs = base.transpose() * (prev_P - prev_T0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t = Lhs_inv * rhs;
        Float t1 = t(0);
        Float t2 = t(1);
        Float t0 = 1 - t1 - t2;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt + t2 * (T2 - prev_T2) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 12> GradV = Eigen::Matrix<Float, 6, 12>::Zero();
        GradV.block<3, 3>(0, 0) = I / dt;
        GradV.block<3, 3>(3, 3) = I * t0 / dt;
        GradV.block<3, 3>(3, 6) = I * t1 / dt;
        GradV.block<3, 3>(3, 9) = I * t2 / dt;
        Eigen::Matrix<Float, 12, 6> GradV_transpose = GradV.transpose();
        G = GradV_transpose * dFdV;

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
        using namespace muda::distance;
        Float D;
        edge_edge_distance(prev_P0, prev_P1, prev_Q0, prev_Q1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        edge_edge_distance_gradient(prev_P0, prev_P1, prev_Q0, prev_Q1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Vector3 n =  (prev_P0 - prev_P1).cross(prev_Q0 - prev_Q1);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;

        // suppose prev_P_project = prev_P0 + t0 * (prev_P1 - prev_P0)
        //         prev_Q_project = prev_Q0 + t1 * (prev_Q1 - prev_Q0)
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0) = prev_P1 - prev_P0;
        base.block<3, 1>(0, 1) = prev_Q1 - prev_Q0;
        Eigen::Matrix<Float, 2, 2> X = Eigen::Matrix<Float, 2, 2>::Zero();
        X(0, 0) = -1;
        X(1, 1) = 1;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base * X;
        Vector2 rhs = base.transpose() * (prev_P0 - prev_Q0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t = Lhs_inv * rhs;
        Float t0 = t(0);
        Float t1 = t(1);

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P0 - prev_P0) * (1 - t0) / dt + (P1 - prev_P1) * t0 / dt;
        V.segment<3>(3) = (Q0 - prev_Q0) * (1 - t1) / dt + (Q1 - prev_Q1) * t1 / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Float lam = -dBdD * (GradD.segment<3>(0) * (1 - t0) + GradD.segment<3>(3) * t0).norm();
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
        using namespace muda::distance;
        Float D;
        edge_edge_distance(prev_P0, prev_P1, prev_Q0, prev_Q1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector12 GradD;
        edge_edge_distance_gradient(prev_P0, prev_P1, prev_Q0, prev_Q1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Vector3 n =  (prev_P0 - prev_P1).cross(prev_Q0 - prev_Q1);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;

        // suppose prev_P_project = prev_P0 + t0 * (prev_P1 - prev_P0)
        //         prev_Q_project = prev_Q0 + t1 * (prev_Q1 - prev_Q0)
        Eigen::Matrix<Float, 3, 2> base;
        base.block<3, 1>(0, 0) = prev_P1 - prev_P0;
        base.block<3, 1>(0, 1) = prev_Q1 - prev_Q0;
        Eigen::Matrix<Float, 2, 2> X = Eigen::Matrix<Float, 2, 2>::Zero();
        X(0, 0) = -1;
        X(1, 1) = 1;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base * X;
        Vector2 rhs = base.transpose() * (prev_P0 - prev_Q0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv(0, 0) = Lhs(1, 1) / det;
        Lhs_inv(0, 1) = -Lhs(0, 1) / det;
        Lhs_inv(1, 0) = -Lhs(1, 0) / det;
        Lhs_inv(1, 1) = Lhs(0, 0) / det;
        Vector2 t = Lhs_inv * rhs;
        Float t0 = t(0);
        Float t1 = t(1);

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P0 - prev_P0) * (1 - t0) / dt + (P1 - prev_P1) * t0 / dt;
        V.segment<3>(3) = (Q0 - prev_Q0) * (1 - t1) / dt + (Q1 - prev_Q1) * t1 / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Float lam = -dBdD * (GradD.segment<3>(0) * (1 - t0) + GradD.segment<3>(3) * t0).norm();

        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 12> GradV = Eigen::Matrix<Float, 6, 12>::Zero();
        GradV.block<3, 3>(0, 0) = I * (1 - t0)/ dt;
        GradV.block<3, 3>(0, 3) = I * t0 / dt;
        GradV.block<3, 3>(3, 6) = I * (1 - t1) / dt;
        GradV.block<3, 3>(3, 9) = I * t1 / dt;
        Eigen::Matrix<Float, 12, 6> GradV_transpose = GradV.transpose();
        G = GradV_transpose * dFdV;

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
        using namespace muda::distance;
        Float D;
        point_edge_distance(prev_P, prev_E0, prev_E1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector9 GradD;
        point_edge_distance_gradient(prev_P, prev_E0, prev_E1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();
        // suppose prev_P0 = t0 * prev_E0 + t1 * prev_E1
        Float t0 = (prev_P - prev_E1).dot(prev_E0 - prev_E1) / (prev_E0 - prev_E1).squaredNorm();
        Float t1 = 1 - t0;

        Vector3 prev_P0 = t0 * prev_E0 + t1 * prev_E1;
        Vector3 n = prev_P0 - prev_P;
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (E0 - prev_E0) / dt + t1 * (E1 - prev_E1) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Float F;
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
        using namespace muda::distance;
        Float D;
        point_edge_distance(prev_P, prev_E0, prev_E1, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector9 GradD = Vector9::Zero();
        point_edge_distance_gradient(prev_P, prev_E0, prev_E1, GradD);

        Float dBdD = 0;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float lam = -dBdD * GradD.head(3).norm();

        // suppose prev_P0 = t0 * prev_E0 + t1 * prev_E1
        Float t0 = (prev_P - prev_E1).dot(prev_E0 - prev_E1) / (prev_E0 - prev_E1).squaredNorm();
        Float t1 = 1 - t0;

        Vector3 prev_P0 = t0 * prev_E0 + t1 * prev_E1;
        Vector3 n = prev_P0 - prev_P;
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (E0 - prev_E0) / dt + t1 * (E1 - prev_E1) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 9> GradV = Eigen::Matrix<Float, 6, 9>::Zero();
        GradV.block<3, 3>(0, 0) = I / dt;
        GradV.block<3, 3>(3, 3) = I * t0 / dt;
        GradV.block<3, 3>(3, 6) = I * t1 / dt;
        Eigen::Matrix<Float, 9, 6> GradV_transpose = GradV.transpose();
        G = GradV_transpose * dFdV;

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
        using namespace muda::distance;
        Float D;
        point_point_distance(prev_P, prev_Q, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector6 GradD;
        point_point_distance_gradient(prev_P, prev_Q, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();

        Vector3 n = prev_Q - prev_P;
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P - prev_P) / dt;
        V.segment<3>(3) = (Q - prev_Q) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Float F;
        FrictionEnergy(F, lam * mu, eps_v, dt, y);
        return F;
    }

    __device__ void PP_friction_gradient_hessian(Vector6&      G,
                                                 Matrix6x6&   H,
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
        using namespace muda::distance;
        Float D;
        point_point_distance(prev_P, prev_Q, D);
        MUDA_ASSERT(D <= squared_d_hat);
        Vector6 GradD;
        point_point_distance_gradient(prev_P, prev_Q, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        Float lam = -dBdD * GradD.head(3).norm();

        Vector3 n = prev_Q - prev_P;
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = (P - prev_P) / dt;
        V.segment<3>(3) = (Q - prev_Q) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        // GradV = Eigen::Matrix<Float, 6, 6>::Identity();
        G = dFdV;

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        H = ddFddV;
    }

}

void IPCSimplexFrictionalContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle energy
    auto PT_count = info.friction_PTs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PT_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs = info.friction_PTs().viewer().name("PTs"),
                Es  = info.friction_PT_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);

                   auto cid_L = contact_ids(PT[0]);
                   auto cid_R = contact_ids(PT[1]);

                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;


                   Float D_hat = d_hat * d_hat;

                   Float          prev_D;
                   const Vector3& prev_P  = prev_Ps(PT[0]);
                   const Vector3& prev_T0 = prev_Ps(PT[1]);
                   const Vector3& prev_T1 = prev_Ps(PT[2]);
                   const Vector3& prev_T2 = prev_Ps(PT[3]);
                   distance::point_triangle_distance(prev_P, prev_T0, prev_T1, prev_T2, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   Float          D;
                   distance::point_triangle_distance(P, T0, T1, T2, D);
                   // NOTE: D can be larger than D_hat

                   // cout << "PT: " << PT.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PT_friction_energy(kappa, D_hat, friction_rate, dt, P, T0, T1, T2, prev_P, prev_T0, prev_T1, prev_T2, eps_v);
               });

    // Compute Edge-Edge energy
    auto EE_count = info.friction_EEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(EE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs = info.friction_EEs().viewer().name("EEs"),
                Es  = info.friction_EE_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v   = info.eps_velocity(),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat(),
                dt      = info.dt()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   auto cid_L = contact_ids(EE[0]);
                   auto cid_R = contact_ids(EE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   const Vector3& prev_E0 = prev_Ps(EE[0]);
                   const Vector3& prev_E1 = prev_Ps(EE[1]);
                   const Vector3& prev_E2 = prev_Ps(EE[2]);
                   const Vector3& prev_E3 = prev_Ps(EE[3]);

                   Float D_hat = d_hat * d_hat;

                   Float prev_D;
                   distance::edge_edge_distance(prev_E0, prev_E1, prev_E2, prev_E3, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& E0 = Ps(EE[0]);
                   const Vector3& E1 = Ps(EE[1]);
                   const Vector3& E2 = Ps(EE[2]);
                   const Vector3& E3 = Ps(EE[3]);

                   // cout << "EE: " << EE.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::EE_friction_energy(kappa, D_hat, friction_rate, dt, E0, E1, E2, E3, prev_E0, prev_E1, prev_E2, prev_E3, eps_v);
               });

    // Compute Point-Edge energy
    auto PE_count = info.friction_PEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs = info.friction_PEs().viewer().name("PEs"),
                Es  = info.friction_PE_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;


                   const Vector3& prev_P  = prev_Ps(PE[0]);
                   const Vector3& prev_E0 = prev_Ps(PE[1]);
                   const Vector3& prev_E1 = prev_Ps(PE[2]);
                   Float          prev_D;
                   distance::point_edge_distance(prev_P, prev_E0, prev_E1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   const Vector3& P  = Ps(PE[0]);
                   const Vector3& E0 = Ps(PE[1]);
                   const Vector3& E1 = Ps(PE[2]);

                   // cout << "PE: " << PE.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PE_friction_energy(kappa, D_hat, friction_rate, dt, P, E0, E1, prev_P, prev_E0, prev_E1, eps_v);
               });

    // Compute Point-Point energy
    auto PP_count = info.friction_PPs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PP_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs = info.friction_PPs().viewer().name("PPs"),
                Es  = info.friction_PP_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P0 = prev_Ps(PP[0]);
                   const Vector3& prev_P1 = prev_Ps(PP[1]);
                   Float          prev_D;
                   distance::point_point_distance(prev_P0, prev_P1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& P0 = Ps(PP[0]);
                   const Vector3& P1 = Ps(PP[1]);
                   Float          D  = D_hat;

                   // cout << "PP: " << PP.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PP_friction_energy(kappa, D_hat, friction_rate, dt, P0, P1, prev_P0, prev_P1, eps_v);
               });
}

void IPCSimplexFrictionalContact::do_assemble(ContactInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_PTs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs = info.friction_PTs().viewer().name("PTs"),
                Gs  = info.friction_PT_gradients().viewer().name("Gs"),
                Hs  = info.friction_PT_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);

                   auto cid_L = contact_ids(PT[0]);
                   auto cid_R = contact_ids(PT[1]);

                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Vector12 G_friction;
                   Matrix12x12 H_friction;
                   sym::ipc_simplex_contact::PT_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   // Gradient check
                   /*
                   Vector3 test;
                   test(0) = 1e-8;
                   test(1) = 1e-8;
                   test(2) = 1e-8;
                   Float E1 = sym::ipc_simplex_contact::PT_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P + test, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   Float E2 = sym::ipc_simplex_contact::PT_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P - test, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   Float numerical_diff = (E1 - E2) / 2;
                     cout << "numerical_diff: " << numerical_diff << "\n";
                   Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1) + G_friction(2) * test(2);
                     cout << "analytical_diff: " << analytical_diff << "\n";
                   cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff << "\n";
                   cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff << "\n";

                   Vector12 G_friction1 = Eigen::Matrix<Float, 12, 1>::Zero();
                   Vector12 G_friction2 = Eigen::Matrix<Float, 12, 1>::Zero();
                   Matrix12x12 H_friction0 = Eigen::Matrix<Float, 12, 12>::Zero();
                   Vector12 test12 = Eigen::Matrix<Float, 12, 1>::Zero();
                   test12(0) = 1e-8;
                   test12(1) = 1e-8;
                   test12(2) = 1e-8;
                   sym::ipc_simplex_contact::PT_friction_gradient_hessian(
                              G_friction1, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P + test12.segment<3>(0), T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   sym::ipc_simplex_contact::PT_friction_gradient_hessian(
                              G_friction2, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P - test12.segment<3>(0), T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);

                   Vector12 G_friction_numerical_diff = (G_friction1 - G_friction2) / 2;
                   Vector12 G_friction_analytical_diff = H_friction * test12;
                   cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
                   cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
                   cout << "PT_grad_diff: " << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
                   */
                   Gs(i) = G_friction;
                   Hs(i) = H_friction;
               });

    // Compute Edge-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_EEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs = info.friction_EEs().viewer().name("EEs"),
                Gs  = info.friction_EE_gradients().viewer().name("Gs"),
                Hs  = info.friction_EE_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v   = info.eps_velocity(),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat(),
                dt      = info.dt()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   auto cid_L = contact_ids(EE[0]);
                   auto cid_R = contact_ids(EE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_E0 = prev_Ps(EE[0]);
                   const Vector3& prev_E1 = prev_Ps(EE[1]);
                   const Vector3& prev_E2 = prev_Ps(EE[2]);
                   const Vector3& prev_E3 = prev_Ps(EE[3]);

                   Float prev_D;
                   distance::edge_edge_distance(prev_E0, prev_E1, prev_E2, prev_E3, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& E0 = Ps(EE[0]);
                   const Vector3& E1 = Ps(EE[1]);
                   const Vector3& E2 = Ps(EE[2]);
                   const Vector3& E3 = Ps(EE[3]);

                   Vector12 G_friction;
                   Matrix12x12 H_friction;
                   sym::ipc_simplex_contact::EE_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, E0, E1, E2, E3, prev_E0, prev_E1, prev_E2, prev_E3, eps_v);
                   // Gradient check
                   /*
                   Vector3 test;
                   test(0) = 1e-6;
                   test(1) = 1e-6;
                   test(2) = 1e-6;
                   Float E1_ = sym::ipc_simplex_contact::EE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, E0 + test, E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);
                   Float E2_ = sym::ipc_simplex_contact::EE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, E0 - test, E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);
                   Float numerical_diff = (E1_ - E2_) / 2;
                     cout << "numerical_diff: " << numerical_diff << "\n";
                   Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1) + G_friction(2) * test(2);
                     cout << "analytical_diff: " << analytical_diff << "\n";
                   cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff << "\n";
                   cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff << "\n";

                   Vector12 G_friction1 = Eigen::Matrix<Float, 12, 1>::Zero();
                   Vector12 G_friction2 = Eigen::Matrix<Float, 12, 1>::Zero();
                   Matrix12x12 H_friction0 = Eigen::Matrix<Float, 12, 12>::Zero();
                   Vector12 test12 = Eigen::Matrix<Float, 12, 1>::Zero();
                   test12(0) = 1e-8;
                   test12(1) = 1e-8;
                   test12(2) = 1e-8;
                   sym::ipc_simplex_contact::EE_friction_gradient_hessian(
                              G_friction1, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, E0 + test12.segment<3>(0), E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);
                   sym::ipc_simplex_contact::EE_friction_gradient_hessian(
                              G_friction2, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, E0 - test12.segment<3>(0), E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);

                   Vector12 G_friction_numerical_diff = (G_friction1 - G_friction2) / 2;
                   Vector12 G_friction_analytical_diff = H_friction * test12;
                   cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
                   cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
                   cout << "EE_grad_diff: " << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
                   */
                   Gs(i) = G_friction;
                   Hs(i) = H_friction;
               });

    // Compute Point-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_PEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs = info.friction_PEs().viewer().name("PEs"),
                Gs  = info.friction_PE_gradients().viewer().name("Gs"),
                Hs  = info.friction_PE_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P  = prev_Ps(PE[0]);
                   const Vector3& prev_E0 = prev_Ps(PE[1]);
                   const Vector3& prev_E1 = prev_Ps(PE[2]);
                   Float          prev_D;
                   distance::point_edge_distance(prev_P, prev_E0, prev_E1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   const Vector3& P  = Ps(PE[0]);
                   const Vector3& E0 = Ps(PE[1]);
                   const Vector3& E1 = Ps(PE[2]);

                   Vector9 G_friction = Eigen::Matrix<Float, 9, 1>::Zero();
                   Matrix9x9 H_friction = Eigen::Matrix<Float, 9, 9>::Zero();
                   sym::ipc_simplex_contact::PE_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P, E0, E1, prev_P, prev_E0, prev_E1, eps_v);
                   // Gradient check
                   /*
                   Vector3 test;
                   test(0) = 1e-6;
                   test(1) = 1e-6;
                   test(2) = 1e-6;
                   Float E1_ = sym::ipc_simplex_contact::PE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P + test, E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);
                   Float E2_ = sym::ipc_simplex_contact::PE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P - test, E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);
                   Float numerical_diff = (E1_ - E2_) / 2;
                     cout << "numerical_diff: " << numerical_diff << "\n";
                   Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1) + G_friction(2) * test(2);
                     cout << "analytical_diff: " << analytical_diff << "\n";
                   cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff << "\n";
                   cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff << "\n";

                   Vector9 G_friction1 = Eigen::Matrix<Float, 9, 1>::Zero();
                   Vector9 G_friction2 = Eigen::Matrix<Float, 9, 1>::Zero();
                   Matrix9x9 H_friction0 = Eigen::Matrix<Float, 9, 9>::Zero();
                   Vector9 test9 = Eigen::Matrix<Float, 9, 1>::Zero();
                   test9(0) = 1e-8;
                   test9(1) = 1e-8;
                   test9(2) = 1e-8;
                   sym::ipc_simplex_contact::PE_friction_gradient_hessian(
                              G_friction1, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P + test9.segment<3>(0), E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);
                   sym::ipc_simplex_contact::PE_friction_gradient_hessian(
                              G_friction2, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P - test9.segment<3>(0), E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);

                   Vector9 G_friction_numerical_diff = (G_friction1 - G_friction2) / 2;
                   Vector9 G_friction_analytical_diff = H_friction * test9;
                   cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
                   cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
                   cout << "E1_: " << E1_ << "\n";
                   cout << "G_friction: " << G_friction.norm() << "\n";
                   cout << "H_friction: " << H_friction.norm() << "\n";
                   cout << "PE_grad_diff: " << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
                   */
                   Gs(i) = G_friction;
                   Hs(i) = H_friction;
               });

    // Compute Point-Point Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_PPs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs = info.friction_PPs().viewer().name("PPs"),
                Gs  = info.friction_PP_gradients().viewer().name("Gs"),
                Hs  = info.friction_PP_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P0 = prev_Ps(PP[0]);
                   const Vector3& prev_P1 = prev_Ps(PP[1]);
                   Float          prev_D;
                   distance::point_point_distance(prev_P0, prev_P1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   const Vector3& P0 = Ps(PP[0]);
                   const Vector3& P1 = Ps(PP[1]);
                   Float          D  = D_hat;
                   distance::point_point_distance(P0, P1, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction
                   Vector6 G_friction;
                   Matrix6x6 H_friction;
                   sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P0, P1, prev_P0, prev_P1, eps_v);
                   // Gradient check

                //    Vector3 test;
                //    test(0) = 1e-6;
                //    test(1) = 1e-6;
                //    test(2) = 1e-6;
                //    Float E1_ = sym::ipc_simplex_contact::PP_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P0 + test, P1, prev_P0, prev_P1, eps_v);
                //    Float E2_ = sym::ipc_simplex_contact::PP_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P0 - test, P1, prev_P0, prev_P1, eps_v);
                //    Float numerical_diff = (E1_ - E2_) / 2;
                //      cout << "numerical_diff: " << numerical_diff << "\n";
                //    Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1) + G_friction(2) * test(2);
                //      cout << "analytical_diff: " << analytical_diff << "\n";
                //    cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff << "\n";
                //    cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff << "\n";

                //    Vector6 G_friction1 = Eigen::Matrix<Float, 6, 1>::Zero();
                //    Vector6 G_friction2 = Eigen::Matrix<Float, 6, 1>::Zero();
                //    Matrix6x6 H_friction0 = Eigen::Matrix<Float, 6, 6>::Zero();
                //    Vector6 test6 = Eigen::Matrix<Float, 6, 1>::Zero();
                //    test6(0) = 1e-8;
                //    test6(1) = 1e-8;
                //    test6(2) = 1e-8;
                //    sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                //               G_friction1, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P0 + test6.segment<3>(0), P1, prev_P0, prev_P1, eps_v);
                //    sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                //               G_friction2, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P0 - test6.segment<3>(0), P1, prev_P0, prev_P1, eps_v);

                //    Vector6 G_friction_numerical_diff = (G_friction1 - G_friction2) / 2;
                //    Vector6 G_friction_analytical_diff = H_friction * test6;
                //    cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
                //    cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
                //    cout << "EE_grad_diff: " << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";

                   Gs(i) = G_friction;
                   Hs(i) = H_friction;
               });
}
}  // namespace uipc::backend::cuda