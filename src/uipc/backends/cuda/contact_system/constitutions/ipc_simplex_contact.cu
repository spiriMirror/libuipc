#include <contact_system/constitutions/ipc_simplex_contact.h>
#include <muda/ext/geo/distance.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexContact);

void IPCSimplexContact::do_build(BuildInfo& info)
{
}

namespace sym::ipc_simplex_contact
{
#include "sym/ipc_contact.inl"

    __device__ Float PT_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P,
                                       const Vector3& T0,
                                       const Vector3& T1,
                                       const Vector3& T2)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D;
        point_triangle_distance(P, T0, T1, T2, D);
        Float B;
        KappaBarrier(B, kappa, D, D_hat);
        return B;
    }

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

        Vector12 GradD;
        point_triangle_distance_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float lam = -dBdD * GradD.head(3).norm();
        Vector3 n =  (T0 - T1).cross(T0 - T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block(0, 0, 3, 3) = I - normal * normal.transpose();
        Tk.block(3, 0, 3, 3) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        // suppose P0 = t(0) * T0 + t(1) * T1 + t(2) * T2
        Eigen::Matrix<Float, 3, 2> base;
        base << T1 - T0, T2 - T0;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base;
        Vector2 rhs = base.transpose() * (P - T0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv << Lhs(1, 1) / det, -Lhs(0, 1) / det, -Lhs(1, 0) / det, Lhs(0, 0) / det;
        Vector2 t = Lhs_inv * rhs;
        Float t1 = t(0);
        Float t2 = t(1);
        Float t0 = 1 - t1 - t2;

        Eigen::Vector<Float, 6> V;
        V  << v1, t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt + t2 * (T2 - prev_T2) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        // cout << "lam: " << lam << "\n";
        // cout << "y: " << y << "\n";
        Float F;
        FrictionEnergy(F, lam * mu, eps_v, dt, y);
        return F;
    }

    __device__ void PT_barrier_gradient_hessian(Vector12&      G,
                                                Matrix12x12&   H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& P,
                                                const Vector3& T0,
                                                const Vector3& T1,
                                                const Vector3& T2)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D;
        point_triangle_distance(P, T0, T1, T2, D);

        Vector12 GradD;
        point_triangle_distance_gradient(P, T0, T1, T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix12x12 HessD;
        point_triangle_distance_hessian(P, T0, T1, T2, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
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

        Vector12 GradD;
        point_triangle_distance_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float lam = -dBdD * GradD.head(3).norm();
        Vector3 n =  (T0 - T1).cross(T0 - T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block(0, 0, 3, 3) = I - normal * normal.transpose();
        Tk.block(3, 0, 3, 3) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        // suppose P0 = t(0) * T0 + t(1) * T1 + t(2) * T2
        Eigen::Matrix<Float, 3, 2> base;
        base << T1 - T0, T2 - T0;
        Eigen::Matrix<Float, 2, 2> Lhs = base.transpose() * base;
        Vector2 rhs = base.transpose() * (P - T0);
        Eigen::Matrix<Float, 2, 2> Lhs_inv;
        Float det = Lhs(0, 0) * Lhs(1, 1) - Lhs(0, 1) * Lhs(1, 0);
        Lhs_inv << Lhs(1, 1) / det, -Lhs(0, 1) / det, -Lhs(1, 0) / det, Lhs(0, 0) / det;
        Vector2 t = Lhs_inv * rhs;
        Float t1 = t(0);
        Float t2 = t(1);
        Float t0 = 1 - t1 - t2;
        Eigen::Vector<Float, 6> V;
        V << v1, t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt + t2 * (T2 - prev_T2) / dt;
        for (int i = 0; i < 6; i++) {
            // cout << "V(" << i << "): " << V(i) << "\n";
        }
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        cout << "y: " << y << "\n";
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Vector3 test;
        test << 1e-10, 1e-10, 1e-10;
        Float E1 = PT_friction_energy(kappa, squared_d_hat, mu, dt, P + test, T0, T1, T2, prev_P, prev_T0, prev_T1, prev_T2, eps_v);
        Float E2 = PT_friction_energy(kappa, squared_d_hat, mu, dt, P - test, T0, T1, T2, prev_P, prev_T0, prev_T1, prev_T2, eps_v);
        Float num_diff = (E1 - E2) / 2;
        for (int i = 0; i < 6; i++) {
            cout << "dFdV(" << i << "): " << dFdV(i) << "\n";
        }
        Vector6 test6;
        test6 << 1e-8, 1e-8, 1e-8, 0, 0, 0;
        Float E3;
        Float y1 = (Tk.transpose() * (V + test6)).norm() * dt;
        cout << "lam_: " << lam << "\n";
        cout << "y1: " << y1 << "\n";
        FrictionEnergy(E3, lam * mu, eps_v, dt, y1);
        Float E4;
        Float y2 = (Tk.transpose() * (V - test6)).norm() * dt;
        cout << "y2: " << y2 << "\n";
        FrictionEnergy(E4, lam * mu, eps_v, dt, y2);
        cout << "E1: " << E1 << "\n";
        cout << "E2: " << E2 << "\n";
        cout << "E3: " << E3 << "\n";
        cout << "E4: " << E4 << "\n";
        Eigen::Matrix<Float, 6, 12> GradV;
        GradV.block(0, 0, 3, 3) = I / dt;
        GradV.block(3, 3, 3, 3) = I * t0 / dt;
        GradV.block(3, 6, 3, 3) = I * t1 / dt;
        GradV.block(3, 9, 3, 3) = I * t2 / dt;
        Eigen::Matrix<Float, 12, 6> GradV_transpose = GradV.transpose();
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 6; j++) {
            // cout << "GradV_transpose(" << i << ", " << j << "): " << GradV_transpose(i, j) << "\n";
            }
        }
        
        for (int i = 0; i < 12; i++) {
            G(i) = 0;
            for (int j = 0; j < 6; j++) {
                G(i) += GradV_transpose(i, j) * dFdV(j);
                cout << "G(" << i << "): " << G(i) << "\n";
            }
        }
        Float ana_diff = G(0) * test(0) + G(1) * test(1) + G(2) * test(2);
        Float ana_diff1 = dFdV.dot(test6);
        Float num_diff1 = (E3 - E4) / 2;
        cout << "ana_diff1: " << ana_diff1 << "\n";
        cout << "num_diff1: " << num_diff1 << "\n";
        cout << "num_diff: " << num_diff << "\n";
        cout << "ana_diff: " << ana_diff << "\n";
        cout << "num_diff - ana_diff: " << num_diff - ana_diff << "\n";
        // G = GradV_transpose * dFdV;
        for (int i = 0; i < 6; i++) {
            // cout << "G_(" << i << "): " << G(i) << "\n";
        }
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(i): " << dFdV(i) << "\n";
        }

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        Vector3 offset = T0 - P + base * t;
        Eigen::Matrix<Float, 2, 3> dtdP = Lhs_inv * base.transpose();
        Eigen::Matrix<Float, 2, 3> dtdT1 = -Lhs_inv.col(0) * offset.transpose() - t1 * dtdP;
        Eigen::Matrix<Float, 2, 3> dtdT2 = -Lhs_inv.col(1) * offset.transpose() - t2 * dtdP;
        Eigen::Matrix<Float, 2, 3> dtdT0 = -dtdP - dtdT1 - dtdT2;
        Vector3 dt0dP = (-dtdP.row(0) - dtdP.row(1)).transpose();
        Vector3 dt0dT1 = (-dtdT1.row(0) - dtdT1.row(1)).transpose();
        Vector3 dt0dT2 = (-dtdT2.row(0) - dtdT2.row(1)).transpose();
        Vector3 dt0dT0 = (-dtdT0.row(0) - dtdT0.row(1)).transpose();
        // Grad(GradV.transpose()) * dFdV
        H.block(0, 0, 3, 12) = Eigen::Matrix<Float, 3, 12>::Zero();
        H.block(3, 0, 3, 3) = dFdV.tail(3) * dt0dP.transpose() / dt;
        H.block(3, 3, 3, 3) = dFdV.tail(3) * dt0dT0.transpose() / dt;
        H.block(3, 6, 3, 3) = dFdV.tail(3) * dt0dT1.transpose() / dt;
        H.block(3, 9, 3, 3) = dFdV.tail(3) * dt0dT2.transpose() / dt;
        H.block(6, 0, 3, 3) = dFdV.tail(3) * dtdP.row(0) / dt;
        H.block(6, 3, 3, 3) = dFdV.tail(3) * dtdT0.row(0) / dt;
        H.block(6, 6, 3, 3) = dFdV.tail(3) * dtdT1.row(0) / dt;
        H.block(6, 9, 3, 3) = dFdV.tail(3) * dtdT2.row(0) / dt;
        H.block(9, 0, 3, 3) = dFdV.tail(3) * dtdP.row(1) / dt;
        H.block(9, 3, 3, 3) = dFdV.tail(3) * dtdT0.row(1) / dt;
        H.block(9, 6, 3, 3) = dFdV.tail(3) * dtdT1.row(1) / dt;
        H.block(9, 9, 3, 3) = dFdV.tail(3) * dtdT2.row(1) / dt;
        cout << "H(0,0): " << H(0, 0) << "\n";
        for (int i=0; i < 6; i++) {
            for (int j=0; j < 6; j++) {
                //cout << "ddFddV(" << i << ", " << j << "): " << ddFddV(i, j) << "\n";
            }
        }
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 12; j++) {
                // cout << "GradV(" << i << ", " << j << "): " << GradV(i, j) << "\n";
            }
        }
        Eigen::Matrix<Float, 6, 12> mat = Eigen::Matrix<Float, 6, 12>::Zero();
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 12; j++) {
                for (int k = 0; k < 6; k++) {
                    mat(i, j) += ddFddV(i, k) * GradV(k, j);
                }
            }
        }
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
               for (int k = 6; k < 6; k++) {
                   H(i, j) += GradV.transpose()(k, i) * mat(k, j);
               }
            }
        }
        cout << "H+(0, 0): " << H(0, 0) << "\n";
    }

    __device__ Float EE_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& t0_Ea0,
                                       const Vector3& t0_Ea1,
                                       const Vector3& t0_Eb0,
                                       const Vector3& t0_Eb1,
                                       const Vector3& Ea0,
                                       const Vector3& Ea1,
                                       const Vector3& Eb0,
                                       const Vector3& Eb1)
    {
        // using mollifier to improve the smoothness of the edge-edge barrier

        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);
        Float B;
        KappaBarrier(B, kappa, D, D_hat);

        Float eps_x;
        edge_edge_mollifier_threshold(t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, eps_x);

        Float ek;
        edge_edge_mollifier(Ea0, Ea1, Eb0, Eb1, eps_x, ek);

        return ek * B;
    }


    __device__ void EE_barrier_gradient_hessian(Vector12&      G,
                                                Matrix12x12&   H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& t0_Ea0,
                                                const Vector3& t0_Ea1,
                                                const Vector3& t0_Eb0,
                                                const Vector3& t0_Eb1,
                                                const Vector3& Ea0,
                                                const Vector3& Ea1,
                                                const Vector3& Eb0,
                                                const Vector3& Eb1)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;


        Float D;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);

        Float B;
        KappaBarrier(B, kappa, D, D_hat);

        //tex: $$ \epsilon_x $$
        Float eps_x;
        edge_edge_mollifier_threshold(t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, eps_x);

        //tex: $$ e_k $$
        Float ek;
        edge_edge_mollifier(Ea0, Ea1, Eb0, Eb1, eps_x, ek);

        //tex: $$\nabla e_k$$
        Vector12 Gradek;
        edge_edge_mollifier_gradient(Ea0, Ea1, Eb0, Eb1, eps_x, Gradek);

        //tex: $$ \nabla D$$
        Vector12 GradD;
        edge_edge_distance_gradient(Ea0, Ea1, Eb0, Eb1, GradD);

        //tex: $$ \frac{\partial B}{\partial D} $$
        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, D_hat);

        //tex: $$ \nabla B = \frac{\partial B}{\partial D} \nabla D$$
        Vector12 GradB = dBdD * GradD;

        //tex:
        //$$
        // G = \nabla e_k B + e_k \nabla B
        //$$
        G = Gradek * B + ek * GradB;

        //tex: $$ \frac{\partial^2 B}{\partial D^2} $$
        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        //tex: $$ \nabla^2 D$$
        Matrix12x12 HessD;
        edge_edge_distance_hessian(Ea0, Ea1, Eb0, Eb1, HessD);

        //tex:
        //$$
        // \nabla^2 B = \frac{\partial^2 B}{\partial D^2} \nabla D \nabla D^T + \frac{\partial B}{\partial D} \nabla^2 D
        //$$
        Matrix12x12 HessB = ddBddD * GradD * GradD.transpose() + dBdD * HessD;

        //tex: $$ \nabla^2 e_k$$
        Matrix12x12 Hessek;
        edge_edge_mollifier_hessian(Ea0, Ea1, Eb0, Eb1, eps_x, Hessek);

        //tex: $$ \nabla^2 e_k B + \nabla e_k \nabla B^T + \nabla B \nabla e_k^T + e_k \nabla^2 B$$
        H = Hessek * B + Gradek * GradB.transpose() + GradB * Gradek.transpose() + ek * HessB;
    }

    __device__ Float PE_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P,
                                       const Vector3& E0,
                                       const Vector3& E1)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_edge_distance(P, E0, E1, D);
        Float E = 0.0;
        KappaBarrier(E, kappa, D, D_hat);
        return E;
    }

    __device__ void PE_barrier_gradient_hessian(Vector9&       G,
                                                Matrix9x9&     H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& P,
                                                const Vector3& E0,
                                                const Vector3& E1)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_edge_distance(P, E0, E1, D);

        Vector9 GradD;
        point_edge_distance_gradient(P, E0, E1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix9x9 HessD;
        point_edge_distance_hessian(P, E0, E1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }

    __device__ Float PP_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P0,
                                       const Vector3& P1)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_point_distance(P0, P1, D);
        Float E = 0.0;
        KappaBarrier(E, kappa, D, D_hat);
        return E;
    }

    __device__ void PP_barrier_gradient_hessian(Vector6&       G,
                                                Matrix6x6&     H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& P0,
                                                const Vector3& P1)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_point_distance(P0, P1, D);

        Vector6 GradD;
        point_point_distance_gradient(P0, P1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix6x6 HessD;
        point_point_distance_hessian(P0, P1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }
}  // namespace sym::ipc_simplex_contact


void IPCSimplexContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle energy
    auto PT_count = info.PTs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PT_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs = info.PTs().viewer().name("PTs"),
                Es  = info.PT_energies().viewer().name("Es"),
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
                   Float D     = D_hat;
                   distance::point_triangle_distance(P, T0, T1, T2, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "PT[%d,%d,%d,%d] d^2(%f) out of range, (0,%f)",
                               PT(0),
                               PT(1),
                               PT(2),
                               PT(3),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::PT_barrier_energy(
                       kappa, D_hat, P, T0, T1, T2);

                   //cout << "PT energy: " << Es(i) << "\n";
               });

    // Compute Edge-Edge energy
    auto EE_count = info.EEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(EE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs = info.EEs().viewer().name("EEs"),
                Es  = info.EE_energies().viewer().name("Es"),
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

                   const auto& E0 = Ps(EE[0]);
                   const auto& E1 = Ps(EE[1]);
                   const auto& E2 = Ps(EE[2]);
                   const auto& E3 = Ps(EE[3]);

                   const auto& t0_Ea0 = rest_Ps(EE[0]);
                   const auto& t0_Ea1 = rest_Ps(EE[1]);
                   const auto& t0_Eb0 = rest_Ps(EE[2]);
                   const auto& t0_Eb1 = rest_Ps(EE[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "EE[%d,%d,%d,%d] d^2(%f) out of range, (0,%f)",
                               EE(0),
                               EE(1),
                               EE(2),
                               EE(3),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::EE_barrier_energy(
                       kappa, D_hat, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
               });

    // Compute Point-Edge energy
    auto PE_count = info.PEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs = info.PEs().viewer().name("PEs"),
                Es  = info.PE_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   const auto& P  = Ps(PE[0]);
                   const auto& E0 = Ps(PE[1]);
                   const auto& E1 = Ps(PE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_edge_distance(P, E0, E1, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "PE[%d,%d,%d] d^2(%f) out of range, (0,%f)",
                               PE(0),
                               PE(1),
                               PE(2),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::PE_barrier_energy(kappa, D_hat, P, E0, E1);
               });

    // Compute Point-Point energy
    auto PP_count = info.PPs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PP_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs = info.PPs().viewer().name("PPs"),
                Es  = info.PP_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   const auto& P0 = Ps(PP[0]);
                   const auto& P1 = Ps(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_point_distance(P0, P1, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "PP[%d,%d] d^2(%f) out of range, (0,%f)",
                               PP(0),
                               PP(1),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::PP_barrier_energy(kappa, D_hat, P0, P1);
               });
}

void IPCSimplexContact::do_assemble(ContactInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PTs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs = info.PTs().viewer().name("PTs"),
                Gs  = info.PT_gradients().viewer().name("Gs"),
                Hs  = info.PT_hessians().viewer().name("Hs"),
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

                   Vector12 G_contact;
                   Vector12 G_friction;
                   Matrix12x12 H_contact;
                   Matrix12x12 H_friction;
                   sym::ipc_contact::PT_barrier_gradient_hessian(
                       G_contact, H_contact, kappa, d_hat * d_hat, P, T0, T1, T2);
                   Vector3 test;
                   test << 1e-6, 1e-6, 1e-6;
                   Float E1 = sym::ipc_contact::PT_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P + test, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   Float E2 = sym::ipc_contact::PT_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P - test, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);

                   sym::ipc_contact::PT_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   for (int j = 0; j < 12; ++j)
                   {
                       // cout << "G_contact(" << j << "): " << G_friction(j) << "\n";
                       for (int k = 0; k < 12; ++k)
                       {
                           // cout << "H_contact(" << j << ", " << k << "): " << H_friction(j, k) << "\n";
                       }
                   }
                   Float numerical_diff = (E1 - E2) / 2;
                     cout << "numerical_diff: " << numerical_diff << "\n";
                   Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1) + G_friction(2) * test(2);
                     cout << "analytical_diff: " << analytical_diff << "\n";
                   cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff << "\n";
                   cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff << "\n";

                   Gs(i) = G_contact + G_friction;
                   // Hs(i) = H_contact + H_friction;
                   Hs(i) = Eigen::Matrix<Float, 12, 12>::Identity();
               });

    // Compute Edge-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.EEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs = info.EEs().viewer().name("EEs"),
                Gs  = info.EE_gradients().viewer().name("Gs"),
                Hs  = info.EE_hessians().viewer().name("Hs"),
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

                   const auto& E0 = Ps(EE[0]);
                   const auto& E1 = Ps(EE[1]);
                   const auto& E2 = Ps(EE[2]);
                   const auto& E3 = Ps(EE[3]);

                   const auto& t0_Ea0 = rest_Ps(EE[0]);
                   const auto& t0_Ea1 = rest_Ps(EE[1]);
                   const auto& t0_Eb0 = rest_Ps(EE[2]);
                   const auto& t0_Eb1 = rest_Ps(EE[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   sym::ipc_simplex_contact::EE_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
               });

    // Compute Point-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs = info.PEs().viewer().name("PEs"),
                Gs  = info.PE_gradients().viewer().name("Gs"),
                Hs  = info.PE_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   const auto& P  = Ps(PE[0]);
                   const auto& E0 = Ps(PE[1]);
                   const auto& E1 = Ps(PE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   sym::ipc_simplex_contact::PE_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P, E0, E1);
               });

    // Compute Point-Point Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PPs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs = info.PPs().viewer().name("PPs"),
                Gs  = info.PP_gradients().viewer().name("Gs"),
                Hs  = info.PP_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   const auto& P0 = Ps(PP[0]);
                   const auto& P1 = Ps(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   sym::ipc_simplex_contact::PP_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P0, P1);
               });
}
}  // namespace uipc::backend::cuda