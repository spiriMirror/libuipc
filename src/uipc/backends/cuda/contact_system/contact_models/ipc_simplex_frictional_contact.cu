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
        Float D_now;
        point_triangle_distance(P, T0, T1, T2, D_now);
        // cout << "D - squared_d_hat: " << D - squared_d_hat << "\n";
        // cout << "D_now - squared_d_hat: " << D_now - squared_d_hat << "\n";
        if (D >= squared_d_hat) {
            return 0;
        }
        Vector12 GradD;
        point_triangle_distance_gradient(prev_P, prev_T0, prev_T1, prev_T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        // cout << "dBdD: " << dBdD << "\n";
        // cout << "GradD.head(3).norm(): " << GradD.head(3).norm() << "\n";
        // cout << "D: " << D << "\n";
        // cout << "D_now: " << D_now << "\n";
        // cout << "squared_d_hat: " << squared_d_hat << "\n";
        Float lam = -dBdD * GradD.head(3).norm();
        Vector3 n =  (prev_T0 - prev_T1).cross(prev_T0 - prev_T2);
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;
        // cout << "v1.norm(): " << v1.norm() << "\n";

        // suppose P0 = t(0) * T0 + t(1) * T1 + t(2) * T2
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
        cout << "PT_lam: " << lam << "\n";
        cout << "PT_y: " << y << "\n";
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
        if (D >= squared_d_hat) {
            G.setZero();
            H = Matrix12x12::Zero();
            return;
        }
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

        // suppose prev_P0 = t(0) * prev_T0 + t(1) * prev_T1 + t(2) * prev_T2
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
        for (int i = 0; i < 6; i++) {
            // cout << "V(" << i << "): " << V(i) << "\n";
        }
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        // cout << "y: " << y << "\n";
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Vector3 test;
        test(0) = 1e-8;
        test(1) = 1e-8;
        test(2) = 1e-8;
        Float E1 = PT_friction_energy(kappa, squared_d_hat, mu, dt, P + test, T0, T1, T2, prev_P, prev_T0, prev_T1, prev_T2, eps_v);
        Float E2 = PT_friction_energy(kappa, squared_d_hat, mu, dt, P - test, T0, T1, T2, prev_P, prev_T0, prev_T1, prev_T2, eps_v);
        Float num_diff = (E1 - E2) / 2;
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(" << i << "): " << dFdV(i) << "\n";
        }
        Vector6 test6 = Vector6::Zero();
        test6(0) = 1e-8;
        test6(1) = 1e-8;
        test6(2) = 1e-8;
        Float E3;
        Float y1 = (Tk.transpose() * (V + test6)).norm() * dt;
        FrictionEnergy(E3, lam * mu, eps_v, dt, y1);
        Float E4;
        Float y2 = (Tk.transpose() * (V - test6)).norm() * dt;
        FrictionEnergy(E4, lam * mu, eps_v, dt, y2);
        Eigen::Matrix<Float, 6, 12> GradV = Eigen::Matrix<Float, 6, 12>::Zero();
        GradV.block<3, 3>(0, 0) = I / dt;
        GradV.block<3, 3>(3, 3) = I * t0 / dt;
        GradV.block<3, 3>(3, 6) = I * t1 / dt;
        GradV.block<3, 3>(3, 9) = I * t2 / dt;
        Eigen::Matrix<Float, 12, 6> GradV_transpose = GradV.transpose();
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 6; j++) {
            // cout << "GradV_transpose(" << i << ", " << j << "): " << GradV_transpose(i, j) << "\n";
            }
        }
        G = GradV_transpose * dFdV;
        /*
        for (int i = 0; i < 12; i++) {
            G(i) = 0;
            for (int j = 0; j < 6; j++) {
                G(i) += GradV_transpose(i, j) * dFdV(j);
                // cout << "G(" << i << "): " << G(i) << "\n";
            }
        }
        Gradient check
        Float ana_diff = G(0) * test(0) + G(1) * test(1) + G(2) * test(2);
        Float ana_diff1 = dFdV.dot(test6);
        Float num_diff1 = (E3 - E4) / 2;
        cout << "ana_diff1: " << ana_diff1 << "\n";
        cout << "num_diff1: " << num_diff1 << "\n";
        cout << "num_diff: " << num_diff << "\n";
        cout << "ana_diff: " << ana_diff << "\n";
        cout << "num_diff - ana_diff: " << num_diff - ana_diff << "\n";
        */
        // G = GradV_transpose * dFdV;
        for (int i = 0; i < 6; i++) {
            // cout << "G_(" << i << "): " << G(i) << "\n";
        }
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(i): " << dFdV(i) << "\n";
        }

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        /*
        Vector6 dFdV1; 
        Vector3 vk_1 = Tk.transpose() * (V + test6);
        dFrictionEnergydV(dFdV1, lam * mu, Tk, eps_v, dt, vk_1);
        Vector6 dFdV2;
        Vector3 vk_2 = Tk.transpose() * (V - test6); 
        dFrictionEnergydV(dFdV2, lam * mu, Tk, eps_v, dt, vk_2);
        Vector6 num_diff6 = (dFdV1 - dFdV2) / 2;
        Vector6 ana_diff6 = ddFddV * test6;
        cout << "num_diff6: " << num_diff6.norm() << "\n";
        cout << "ana_diff6: " << ana_diff6.norm() << "\n";
        cout << "num_diff6 - ana_diff6: " << (num_diff6 - ana_diff6).norm() << "\n";

        The method is semi-implicit, where the t is explicit t_n compute with previous points, so the gradient dt is all 0.
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
        // a = (a1, a2), b = (b1, b2), c = (c1, c2) 
        // b0 = c0, b1 = b0 * c1
        // da1dc1 = da1
        H.block<3, 12>(0, 0) = Eigen::Matrix<Float, 3, 12>::Zero();
        H.block<3, 3>(3, 0) = dFdV.tail(3) * dt0dP.transpose() / dt;
        H.block<3, 3>(3, 3) = dFdV.tail(3) * dt0dT0.transpose() / dt;
        H.block<3, 3>(3, 6) = dFdV.tail(3) * dt0dT1.transpose() / dt;
        H.block<3, 3>(3, 9) = dFdV.tail(3) * dt0dT2.transpose() / dt;
        H.block<3, 3>(6, 0) = dFdV.tail(3) * dtdP.row(0) / dt;
        H.block<3, 3>(6, 3) = dFdV.tail(3) * dtdT0.row(0) / dt;
        H.block<3, 3>(6, 6) = dFdV.tail(3) * dtdT1.row(0) / dt;
        H.block<3, 3>(6, 9) = dFdV.tail(3) * dtdT2.row(0) / dt;
        H.block<3, 3>(9, 0) = dFdV.tail(3) * dtdP.row(1) / dt;
        H.block<3, 3>(9, 3) = dFdV.tail(3) * dtdT0.row(1) / dt;
        H.block<3, 3>(9, 6) = dFdV.tail(3) * dtdT1.row(1) / dt;
        H.block<3, 3>(9, 9) = dFdV.tail(3) * dtdT2.row(1) / dt;
        cout << "H(0,0): " << H(0, 0) << "\n";
        cout << "H - H.transpose(): " << (H - H.transpose()).norm() << "\n";
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
            }
        }
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
        */
        H.block<3, 3>(0, 0) = ddFddV.block<3, 3>(0, 0) / dt / dt;
        H.block<3, 3>(0, 3) = ddFddV.block<3, 3>(0, 3) * t0 / dt / dt;
        H.block<3, 3>(0, 6) = ddFddV.block<3, 3>(0, 3) * t1 / dt / dt;
        H.block<3, 3>(0, 9) = ddFddV.block<3, 3>(0, 3) * t2 / dt / dt;
        H.block<3, 3>(3, 0) = ddFddV.block<3, 3>(3, 0) * t0 / dt / dt;
        H.block<3, 3>(3, 3) = ddFddV.block<3, 3>(3, 3) * t0 * t0 / dt / dt;
        H.block<3, 3>(3, 6) = ddFddV.block<3, 3>(3, 3) * t0 * t1 / dt / dt;
        H.block<3, 3>(3, 9) = ddFddV.block<3, 3>(3, 3) * t0 * t2 / dt / dt;
        H.block<3, 3>(6, 0) = ddFddV.block<3, 3>(3, 0) * t1 / dt / dt;
        H.block<3, 3>(6, 3) = ddFddV.block<3, 3>(3, 3) * t1 * t0 / dt / dt;
        H.block<3, 3>(6, 6) = ddFddV.block<3, 3>(3, 3) * t1 * t1 / dt / dt;
        H.block<3, 3>(6, 9) = ddFddV.block<3, 3>(3, 3) * t1 * t2 / dt / dt;
        H.block<3, 3>(9, 0) = ddFddV.block<3, 3>(3, 0) * t2 / dt / dt;
        H.block<3, 3>(9, 3) = ddFddV.block<3, 3>(3, 3) * t2 * t0 / dt / dt;
        H.block<3, 3>(9, 6) = ddFddV.block<3, 3>(3, 3) * t2 * t1 / dt / dt;
        H.block<3, 3>(9, 9) = ddFddV.block<3, 3>(3, 3) * t2 * t2 / dt / dt;
        /*
        Eigen::Matrix<Float, 6, 12> mat = Eigen::Matrix<Float, 6, 12>::Zero();
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 12; j++) {
                for (int k = 0; k < 6; k++) {
                    mat(i, j) += ddFddV(i, k) * GradV(k, j);
                    cout << "mat(" << i << ", " << j << "): " << mat(i, j) << "\n";
                }
            }
        }
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                H(i, j) = 0;
                for (int k = 0; k < 6; k++) {
                    H(i, j) += GradV.transpose()(i, k) * mat(k, j);
                    // cout << "GradV.transpose()(" << i << ", " << k << "): " << GradV.transpose()(i, k) << "\n";
                    // cout << "mat(" << k << ", " << j << "): " << mat(k, j) << "\n";
                    cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
               }
            }
        }
        */
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                // cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
            }
        }
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
        Float D_now;
        edge_edge_distance(P0, P1, Q0, Q1, D_now);
        cout << "D - squared_d_hat: " << D - squared_d_hat << "\n";
        // cout << "D_now - squared_d_hat: " << D_now - squared_d_hat << "\n";
        if (D >= squared_d_hat) {
            return 0;
        }
        Vector12 GradD;
        edge_edge_distance_gradient(prev_P0, prev_P1, prev_Q0, prev_Q1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        // cout << "dBdD: " << dBdD << "\n";
        // cout << "GradD.head(3).norm(): " << GradD.head(3).norm() << "\n";
        // cout << "D: " << D << "\n";
        // cout << "D_now: " << D_now << "\n";
        // cout << "squared_d_hat: " << squared_d_hat << "\n";
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
        cout << "EE_lam: " << lam << "\n";
        cout << "EE_y: " << y << "\n";
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
        Float D_now;
        edge_edge_distance(P0, P1, Q0, Q1, D_now);
        // cout << "D - squared_d_hat: " << D - squared_d_hat << "\n";
        // cout << "D_now - squared_d_hat: " << D_now - squared_d_hat << "\n";
        if (D >= squared_d_hat) {
            G.setZero();
            H = Matrix12x12::Zero();
            return;
        }
        Vector12 GradD;
        edge_edge_distance_gradient(prev_P0, prev_P1, prev_Q0, prev_Q1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        // cout << "dBdD: " << dBdD << "\n";
        // cout << "GradD.head(3).norm(): " << GradD.head(3).norm() << "\n";
        // cout << "D: " << D << "\n";
        // cout << "D_now: " << D_now << "\n";
        // cout << "squared_d_hat: " << squared_d_hat << "\n";
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
        // cout << "lam: " << lam << "\n";
        // cout << "y: " << y << "\n";
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Eigen::Matrix<Float, 6, 12> GradV = Eigen::Matrix<Float, 6, 12>::Zero();
        GradV.block<3, 3>(0, 0) = I * (1 - t0)/ dt;
        GradV.block<3, 3>(0, 3) = I * t0 / dt;
        GradV.block<3, 3>(3, 6) = I * (1 - t1) / dt;
        GradV.block<3, 3>(3, 9) = I * t1 / dt;
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
                // cout << "G(" << i << "): " << G(i) << "\n";
            }
        }
        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        H.block<3, 3>(0, 0) = ddFddV.block<3, 3>(0, 0) * (1 - t0) * (1 - t0) / dt / dt;
        H.block<3, 3>(0, 3) = ddFddV.block<3, 3>(0, 3) * (1 - t0) * t0 / dt / dt;
        H.block<3, 3>(0, 6) = ddFddV.block<3, 3>(0, 3) * (1 - t0) * (1 - t1) / dt / dt;
        H.block<3, 3>(0, 9) = ddFddV.block<3, 3>(0, 3) * (1 - t0) * t1 / dt / dt;
        H.block<3, 3>(3, 0) = ddFddV.block<3, 3>(3, 0) * t0 * (1 - t0) / dt / dt;
        H.block<3, 3>(3, 3) = ddFddV.block<3, 3>(3, 3) * t0 * t0 / dt / dt;
        H.block<3, 3>(3, 6) = ddFddV.block<3, 3>(3, 3) * t0 * (1 - t1) / dt / dt;
        H.block<3, 3>(3, 9) = ddFddV.block<3, 3>(3, 3) * t0 * t1 / dt / dt;
        H.block<3, 3>(6, 0) = ddFddV.block<3, 3>(3, 0) * (1 - t1) * (1 - t0) / dt / dt;
        H.block<3, 3>(6, 3) = ddFddV.block<3, 3>(3, 3) * (1 - t1) * t0 / dt / dt;
        H.block<3, 3>(6, 6) = ddFddV.block<3, 3>(3, 3) * (1 - t1) * (1 - t1) / dt / dt;
        H.block<3, 3>(6, 9) = ddFddV.block<3, 3>(3, 3) * (1 - t1) * t1 / dt / dt;
        H.block<3, 3>(9, 0) = ddFddV.block<3, 3>(3, 0) * t1 * (1 - t0) / dt / dt;
        H.block<3, 3>(9, 3) = ddFddV.block<3, 3>(3, 3) * t1 * t0 / dt / dt;
        H.block<3, 3>(9, 6) = ddFddV.block<3, 3>(3, 3) * t1 * (1 - t1) / dt / dt;
        H.block<3, 3>(9, 9) = ddFddV.block<3, 3>(3, 3) * t1 * t1 / dt / dt;
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                // cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
            }
        }
    }

    __device__ Float PE_friction_energy(Float          kappa,
                                        Float          squared_d_hat,
                                        Float          mu,
                                        Float          dt,
                                        const Vector3& P,
                                        const Vector3& T0,
                                        const Vector3& T1,
                                        const Vector3& prev_P,
                                        const Vector3& prev_T0,
                                        const Vector3& prev_T1,
                                        Float          eps_v)
    {
        using namespace muda::distance;
        Float D;
        point_edge_distance(prev_P, prev_T0, prev_T1, D);
        Float D_now;
        point_edge_distance(P, T0, T1, D_now);
        // cout << "D - squared_d_hat: " << D - squared_d_hat << "\n";
        // cout << "D_now - squared_d_hat: " << D_now - squared_d_hat << "\n";
        if (D >= squared_d_hat) {
            return 0;
        }
        Vector9 GradD;
        point_edge_distance_gradient(prev_P, prev_T0, prev_T1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        // cout << "dBdD: " << dBdD << "\n";
        // cout << "GradD.head(3).norm(): " << GradD.head(3).norm() << "\n";
        // cout << "D: " << D << "\n";
        // cout << "D_now: " << D_now << "\n";
        // cout << "squared_d_hat: " << squared_d_hat << "\n";
        Float lam = -dBdD * GradD.head(3).norm();
        cout << "PE_lam: " << lam << "\n";
        // suppose P0 = t(0) * T0 + t(1) * T1
        Float t0 = (prev_P - prev_T1).dot(prev_T0 - prev_T1) / (prev_T0 - prev_T1).squaredNorm();
        Float t1 = 1 - t0;

        Vector3 prev_P0 = t0 * prev_T0 + t1 * prev_T1;
        Vector3 n = prev_P0 - prev_P;
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt;
        Vector3 vk = Tk.transpose() * V;
        Float y = vk.norm() * dt;
        // cout << "lam: " << lam << "\n";
        cout << "PE_y: " << y << "\n";
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
                                                 const Vector3& T0,
                                                 const Vector3& T1,
                                                 const Vector3& prev_P,
                                                 const Vector3& prev_T0,
                                                 const Vector3& prev_T1,
                                                 Float          eps_v)
    {
        using namespace muda::distance;
        Float D;
        point_edge_distance(prev_P, prev_T0, prev_T1, D);
        if (D >= squared_d_hat) {
            MUDA_ASSERT(D >= squared_d_hat);
            G.setZero();
            H = Matrix9x9::Zero();
            return;
        }
        Vector9 GradD = Vector9::Zero();
        point_edge_distance_gradient(prev_P, prev_T0, prev_T1, GradD);

        Float dBdD = 0;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        Float lam = -dBdD * GradD.head(3).norm();

        // suppose P0 = t0 * T0 + t1 * T1
        Float t0 = (prev_P - prev_T1).dot(prev_T0 - prev_T1) / (prev_T0 - prev_T1).squaredNorm();
        Float t1 = 1 - t0;

        Vector3 prev_P0 = t0 * prev_T0 + t1 * prev_T1;
        Vector3 n = prev_P0 - prev_P;
        cout << "t0: " << t0 << "\n";
        cout << "n.dot(prev_T0 - prev_T1): " << n.dot(prev_T0 - prev_T1) << "\n";
        Vector3 normal = n / n.norm();
        Eigen::Matrix<Float, 6, 3> Tk;
        Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
        Tk.block<3, 3>(0, 0) = I - normal * normal.transpose();
        Tk.block<3, 3>(3, 0) = normal * normal.transpose() - I;
        Vector3 v1 = (P - prev_P) / dt;

        Eigen::Vector<Float, 6> V;
        V.segment<3>(0) = v1;
        V.segment<3>(3) = t0 * (T0 - prev_T0) / dt + t1 * (T1 - prev_T1) / dt;
        for (int i = 0; i < 6; i++) {
            // cout << "V(" << i << "): " << V(i) << "\n";
        }
        cout << "V.norm(): " << V.norm() << "\n";
        Vector3 vk = Tk.transpose() * V;
        cout << "vk: " << vk.norm() << "\n";
        Float y = vk.norm() * dt;
        // cout << "y: " << y << "\n";
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        cout << "dFdV: " << dFdV.norm() << "\n";
        Vector3 test;
        test(0) = 1e-8;
        test(1) = 1e-8;
        test(2) = 1e-8;
        Float E1 = PE_friction_energy(kappa, squared_d_hat, mu, dt, P + test, T0, T1, prev_P, prev_T0, prev_T1, eps_v);
        Float E2 = PE_friction_energy(kappa, squared_d_hat, mu, dt, P - test, T0, T1, prev_P, prev_T0, prev_T1, eps_v);
        Float num_diff = (E1 - E2) / 2;
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(" << i << "): " << dFdV(i) << "\n";
        }
        Vector6 test6 = Vector6::Zero();
        test6(0) = 1e-8;
        test6(1) = 1e-8;
        test6(2) = 1e-8;
        Float E3;
        Float y1 = (Tk.transpose() * (V + test6)).norm() * dt;
        FrictionEnergy(E3, lam * mu, eps_v, dt, y1);
        Float E4;
        Float y2 = (Tk.transpose() * (V - test6)).norm() * dt;
        FrictionEnergy(E4, lam * mu, eps_v, dt, y2);
        Eigen::Matrix<Float, 6, 9> GradV = Eigen::Matrix<Float, 6, 9>::Zero();
        GradV.block<3, 3>(0, 0) = I / dt;
        GradV.block<3, 3>(3, 3) = I * t0 / dt;
        GradV.block<3, 3>(3, 6) = I * t1 / dt;
        Eigen::Matrix<Float, 9, 6> GradV_transpose = GradV.transpose();
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 6; j++) {
            // cout << "GradV_transpose(" << i << ", " << j << "): " << GradV_transpose(i, j) << "\n";
            }
        }
        G = GradV_transpose * dFdV;
        cout << "prev_P - P: " << (prev_P - P).norm() << "\n";
        cout << "prev_T0 - T0: " << (prev_T0 - T0).norm() << "\n";
        cout << "prev_T1 - T1: " << (prev_T1 - T1).norm() << "\n";
        cout << "G.norm(): " << G.norm() << "\n";
        /*
        for (int i = 0; i < 12; i++) {
            G(i) = 0;
            for (int j = 0; j < 6; j++) {
                G(i) += GradV_transpose(i, j) * dFdV(j);
                // cout << "G(" << i << "): " << G(i) << "\n";
            }
        }
        Gradient check
        Float ana_diff = G(0) * test(0) + G(1) * test(1) + G(2) * test(2);
        Float ana_diff1 = dFdV.dot(test6);
        Float num_diff1 = (E3 - E4) / 2;
        cout << "ana_diff1: " << ana_diff1 << "\n";
        cout << "num_diff1: " << num_diff1 << "\n";
        cout << "num_diff: " << num_diff << "\n";
        cout << "ana_diff: " << ana_diff << "\n";
        cout << "num_diff - ana_diff: " << num_diff - ana_diff << "\n";
        */
        // G = GradV_transpose * dFdV;
        for (int i = 0; i < 6; i++) {
            // cout << "G_(" << i << "): " << G(i) << "\n";
        }
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(i): " << dFdV(i) << "\n";
        }

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);
        /*
        Vector6 dFdV1; 
        Vector3 vk_1 = Tk.transpose() * (V + test6);
        dFrictionEnergydV(dFdV1, lam * mu, Tk, eps_v, dt, vk_1);
        Vector6 dFdV2;
        Vector3 vk_2 = Tk.transpose() * (V - test6); 
        dFrictionEnergydV(dFdV2, lam * mu, Tk, eps_v, dt, vk_2);
        Vector6 num_diff6 = (dFdV1 - dFdV2) / 2;
        Vector6 ana_diff6 = ddFddV * test6;
        cout << "num_diff6: " << num_diff6.norm() << "\n";
        cout << "ana_diff6: " << ana_diff6.norm() << "\n";
        cout << "num_diff6 - ana_diff6: " << (num_diff6 - ana_diff6).norm() << "\n";

        The method is semi-implicit, where the t is explicit t_n compute with previous points, so the gradient dt is all 0.
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
        // a = (a1, a2), b = (b1, b2), c = (c1, c2) 
        // b0 = c0, b1 = b0 * c1
        // da1dc1 = da1
        H.block<3, 12>(0, 0) = Eigen::Matrix<Float, 3, 12>::Zero();
        H.block<3, 3>(3, 0) = dFdV.tail(3) * dt0dP.transpose() / dt;
        H.block<3, 3>(3, 3) = dFdV.tail(3) * dt0dT0.transpose() / dt;
        H.block<3, 3>(3, 6) = dFdV.tail(3) * dt0dT1.transpose() / dt;
        H.block<3, 3>(3, 9) = dFdV.tail(3) * dt0dT2.transpose() / dt;
        H.block<3, 3>(6, 0) = dFdV.tail(3) * dtdP.row(0) / dt;
        H.block<3, 3>(6, 3) = dFdV.tail(3) * dtdT0.row(0) / dt;
        H.block<3, 3>(6, 6) = dFdV.tail(3) * dtdT1.row(0) / dt;
        H.block<3, 3>(6, 9) = dFdV.tail(3) * dtdT2.row(0) / dt;
        H.block<3, 3>(9, 0) = dFdV.tail(3) * dtdP.row(1) / dt;
        H.block<3, 3>(9, 3) = dFdV.tail(3) * dtdT0.row(1) / dt;
        H.block<3, 3>(9, 6) = dFdV.tail(3) * dtdT1.row(1) / dt;
        H.block<3, 3>(9, 9) = dFdV.tail(3) * dtdT2.row(1) / dt;
        cout << "H(0,0): " << H(0, 0) << "\n";
        cout << "H - H.transpose(): " << (H - H.transpose()).norm() << "\n";
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
            }
        }
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
        */
        H.block<3, 3>(0, 0) = ddFddV.block<3, 3>(0, 0) / dt / dt;
        H.block<3, 3>(0, 3) = ddFddV.block<3, 3>(0, 3) * t0 / dt / dt;
        H.block<3, 3>(0, 6) = ddFddV.block<3, 3>(0, 3) * t1 / dt / dt;
        H.block<3, 3>(3, 0) = ddFddV.block<3, 3>(3, 0) * t0 / dt / dt;
        H.block<3, 3>(3, 3) = ddFddV.block<3, 3>(3, 3) * t0 * t0 / dt / dt;
        H.block<3, 3>(3, 6) = ddFddV.block<3, 3>(3, 3) * t0 * t1 / dt / dt;
        H.block<3, 3>(6, 0) = ddFddV.block<3, 3>(3, 0) * t1 / dt / dt;
        H.block<3, 3>(6, 3) = ddFddV.block<3, 3>(3, 3) * t1 * t0 / dt / dt;
        H.block<3, 3>(6, 6) = ddFddV.block<3, 3>(3, 3) * t1 * t1 / dt / dt;
        /*
        Eigen::Matrix<Float, 6, 12> mat = Eigen::Matrix<Float, 6, 12>::Zero();
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 12; j++) {
                for (int k = 0; k < 6; k++) {
                    mat(i, j) += ddFddV(i, k) * GradV(k, j);
                    cout << "mat(" << i << ", " << j << "): " << mat(i, j) << "\n";
                }
            }
        }
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                H(i, j) = 0;
                for (int k = 0; k < 6; k++) {
                    H(i, j) += GradV.transpose()(i, k) * mat(k, j);
                    // cout << "GradV.transpose()(" << i << ", " << k << "): " << GradV.transpose()(i, k) << "\n";
                    // cout << "mat(" << k << ", " << j << "): " << mat(k, j) << "\n";
                    cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
               }
            }
        }
        */
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                // cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
            }
        }
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
        Float D_now;
        point_point_distance(P, Q, D_now);
        // cout << "D - squared_d_hat: " << D - squared_d_hat << "\n";
        // cout << "D_now - squared_d_hat: " << D_now - squared_d_hat << "\n";
        if (D >= squared_d_hat) {
            return 0;
        }
        Vector6 GradD;
        point_point_distance_gradient(prev_P, prev_Q, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        // cout << "dBdD: " << dBdD << "\n";
        // cout << "GradD.head(3).norm(): " << GradD.head(3).norm() << "\n";
        // cout << "D: " << D << "\n";
        // cout << "D_now: " << D_now << "\n";
        // cout << "squared_d_hat: " << squared_d_hat << "\n";
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
        // cout << "lam: " << lam << "\n";
        // cout << "y: " << y << "\n";
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
        Float D_now;
        point_point_distance(P, Q, D_now);
        if (D >= squared_d_hat) {
            return;
        }
        Vector6 GradD;
        point_point_distance_gradient(prev_P, prev_Q, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);
        // cout << "dBdD: " << dBdD << "\n";
        // cout << "GradD.head(3).norm(): " << GradD.head(3).norm() << "\n";
        // cout << "D: " << D << "\n";
        // cout << "D_now: " << D_now << "\n";
        // cout << "squared_d_hat: " << squared_d_hat << "\n";
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
        // cout << "y: " << y << "\n";
        Eigen::Vector<Float, 6> dFdV;
        dFrictionEnergydV(dFdV, lam * mu, Tk,  eps_v, dt, vk);
        Vector3 test;
        test(0) = 1e-8;
        test(1) = 1e-8;
        test(2) = 1e-8;
        Float E1 = PP_friction_energy(kappa, squared_d_hat, mu, dt, P + test, Q, prev_P, prev_Q, eps_v);
        Float E2 = PP_friction_energy(kappa, squared_d_hat, mu, dt, P - test, Q, prev_P, prev_Q, eps_v);
        Float num_diff = (E1 - E2) / 2;
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(" << i << "): " << dFdV(i) << "\n";
        }
        Vector6 test6 = Vector6::Zero();
        test6(0) = 1e-8;
        test6(1) = 1e-8;
        test6(2) = 1e-8;
        Float E3;
        Float y1 = (Tk.transpose() * (V + test6)).norm() * dt;
        FrictionEnergy(E3, lam * mu, eps_v, dt, y1);
        Float E4;
        Float y2 = (Tk.transpose() * (V - test6)).norm() * dt;
        FrictionEnergy(E4, lam * mu, eps_v, dt, y2);
        // GradV = Eigen::Matrix<Float, 6, 6>::Identity();
        G = dFdV;
        /*
        Gradient check
        Float ana_diff = G(0) * test(0) + G(1) * test(1) + G(2) * test(2);
        Float ana_diff1 = dFdV.dot(test6);
        Float num_diff1 = (E3 - E4) / 2;
        cout << "ana_diff1: " << ana_diff1 << "\n";
        cout << "num_diff1: " << num_diff1 << "\n";
        cout << "num_diff: " << num_diff << "\n";
        cout << "ana_diff: " << ana_diff << "\n";
        cout << "num_diff - ana_diff: " << num_diff - ana_diff << "\n";
        */
        // G = GradV_transpose * dFdV;
        for (int i = 0; i < 6; i++) {
            // cout << "G_(" << i << "): " << G(i) << "\n";
        }
        for (int i = 0; i < 6; i++) {
            // cout << "dFdV(i): " << dFdV(i) << "\n";
        }

        Eigen::Matrix<Float, 6, 6> ddFddV;
        ddFrictionEnergyddV(ddFddV, lam * mu, Tk, eps_v, dt, vk);

        H = ddFddV;

        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                // cout << "H(" << i << ", " << j << "): " << H(i, j) << "\n";
            }
        }
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

                   cout << "PT: " << PT.transpose().eval() << " D/prev_D: " << D
                        << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PT_friction_energy(kappa, D_hat, friction_rate, dt, P, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   // cout << "PT_barrier_energy: " << sym::ipc_simplex_contact::PT_barrier_energy(kappa, D_hat, P, T0, T1, T2) << "\n";
                   // cout << "PT_friction_energy: " << sym::ipc_simplex_contact::PT_friction_energy(kappa, D_hat, friction_rate, dt, P, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v) << "\n";
                   //cout << "PT energy: " << Es(i) << "\n";
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

                   const Vector3& t0_Ea0 = rest_Ps(EE[0]);
                   const Vector3& t0_Ea1 = rest_Ps(EE[1]);
                   const Vector3& t0_Eb0 = rest_Ps(EE[2]);
                   const Vector3& t0_Eb1 = rest_Ps(EE[3]);


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

                   Float D = D_hat;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);
                   // NOTE: D can be larger than D_hat


                   cout << "EE: " << EE.transpose().eval() << " D/prev_D: " << D
                        << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::EE_friction_energy(kappa, D_hat, friction_rate, dt, E0, E1, E2, E3, prev_E0, prev_E1, prev_E2, prev_E3, eps_v);
                   // cout << "sym::ipc_simplex_contact::EE_friction_energy(kappa, D_hat, friction_rate, dt, E0, E1, E2, E3, prev_E0, prev_E1, prev_E2, prev_E3, eps_v): "
                   //      << sym::ipc_simplex_contact::EE_friction_energy(kappa, D_hat, friction_rate, dt, E0, E1, E2, E3, prev_E0, prev_E1, prev_E2, prev_E3, eps_v) << "\n";
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
                   Float          D;
                   distance::point_edge_distance(P, E0, E1, D);
                   // NOTE: D can be larger than D_hat

                   cout << "PE: " << PE.transpose().eval() << " D/prev_D: " << D
                        << "/" << prev_D << "\n";

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
                   distance::point_point_distance(P0, P1, D);
                   // NOTE: D can be larger than D_hat

                   cout << "PP: " << PP.transpose().eval() << " D/prev_D: " << D
                        << "/" << prev_D << "\n";

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
                   Vector3 test;
                   test(0) = 1e-8;
                   test(1) = 1e-8;
                   test(2) = 1e-8;
                   Float E1 = sym::ipc_simplex_contact::PT_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P + test, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);
                   Float E2 = sym::ipc_simplex_contact::PT_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P - test, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);

                   sym::ipc_simplex_contact::PT_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P, T0, T1, T2, prev_Ps(PT[0]), prev_Ps(PT[1]), prev_Ps(PT[2]), prev_Ps(PT[3]), eps_v);

                   // Gradient check
                   /*
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
                    cout << "G_friction: " << G_friction.norm() << "\n";
                    cout << "H_friction: " << H_friction.norm() << "\n";
                   // Hs(i) = Eigen::Matrix<Float, 12, 12>::Identity();
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

                   const Vector3& t0_Ea0 = rest_Ps(EE[0]);
                   const Vector3& t0_Ea1 = rest_Ps(EE[1]);
                   const Vector3& t0_Eb0 = rest_Ps(EE[2]);
                   const Vector3& t0_Eb1 = rest_Ps(EE[3]);

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

                   Float D;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction

                   Vector12 G_friction;
                   Matrix12x12 H_friction;
                   Vector3 test;
                   test(0) = 1e-6;
                   test(1) = 1e-6;
                   test(2) = 1e-6;
                   Float E1_ = sym::ipc_simplex_contact::EE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, E0 + test, E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);
                   Float E2_ = sym::ipc_simplex_contact::EE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, E0 - test, E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);

                   sym::ipc_simplex_contact::EE_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, E0, E1, E2, E3, prev_Ps(EE[0]), prev_Ps(EE[1]), prev_Ps(EE[2]), prev_Ps(EE[3]), eps_v);

                   // Gradient check
                   /*
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
                   cout << "EE_G_friction: " << G_friction.norm() << "\n";
                     cout << "EE_H_friction: " << H_friction.norm() << "\n";
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
                   Float          D;
                   distance::point_edge_distance(P, E0, E1, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction
                   Vector9 G_friction = Eigen::Matrix<Float, 9, 1>::Zero();
                   Matrix9x9 H_friction = Eigen::Matrix<Float, 9, 9>::Zero();
                   Vector3 test;
                   test(0) = 1e-6;
                   test(1) = 1e-6;
                   test(2) = 1e-6;
                   Float E1_ = sym::ipc_simplex_contact::PE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P + test, E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);
                   Float E2_ = sym::ipc_simplex_contact::PE_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P - test, E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);

                   sym::ipc_simplex_contact::PE_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P, E0, E1, prev_Ps(PE[0]), prev_Ps(PE[1]), prev_Ps(PE[2]), eps_v);

                   // Gradient check
                   
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
                   Vector3 test;
                   test(0) = 1e-6;
                   test(1) = 1e-6;
                   test(2) = 1e-6;
                   Float E1_ = sym::ipc_simplex_contact::PP_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P0 + test, P1, prev_P0, prev_P1, eps_v);
                   Float E2_ = sym::ipc_simplex_contact::PP_friction_energy(kappa, d_hat * d_hat, friction_rate, dt, P0 - test, P1, prev_P0, prev_P1, eps_v);

                   sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                          G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P0, P1, prev_P0, prev_P1, eps_v);

                   // Gradient check
                   
                   Float numerical_diff = (E1_ - E2_) / 2;
                     cout << "numerical_diff: " << numerical_diff << "\n";
                   Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1) + G_friction(2) * test(2);
                     cout << "analytical_diff: " << analytical_diff << "\n";
                   cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff << "\n";
                   cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff << "\n";

                   Vector6 G_friction1 = Eigen::Matrix<Float, 6, 1>::Zero();
                   Vector6 G_friction2 = Eigen::Matrix<Float, 6, 1>::Zero();
                   Matrix6x6 H_friction0 = Eigen::Matrix<Float, 6, 6>::Zero();
                   Vector6 test6 = Eigen::Matrix<Float, 6, 1>::Zero();
                   test6(0) = 1e-8;
                   test6(1) = 1e-8;
                   test6(2) = 1e-8;
                   sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                              G_friction1, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P0 + test6.segment<3>(0), P1, prev_P0, prev_P1, eps_v);
                   sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                              G_friction2, H_friction0, kappa, d_hat * d_hat, friction_rate, dt, P0 - test6.segment<3>(0), P1, prev_P0, prev_P1, eps_v);

                   Vector6 G_friction_numerical_diff = (G_friction1 - G_friction2) / 2;
                   Vector6 G_friction_analytical_diff = H_friction * test6;
                   cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
                   cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
                   cout << "EE_grad_diff: " << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";

                   Gs(i) = G_friction;
                   Hs(i) = H_friction;
               });
}
}  // namespace uipc::backend::cuda