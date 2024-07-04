#include <contact_system/constitutions/ipc_simplex_contact.h>
#include <muda/ext/geo/distance.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexContact);

void IPCSimplexContact::do_build(BuildInfo& info) {}


namespace ipc_contact
{
    namespace sym
    {
        using std::log;
#include <contact_system/constitutions/sym/ipc_contact.inl>
    }  // namespace sym


    __device__ Float PT_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P,
                                       const Vector3& T0,
                                       const Vector3& T1,
                                       const Vector3& T2)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_triangle_distance(P, T0, T1, T2, D);
        Float E = 0.0;
        sym::KappaBarrier(E, kappa, D, D_hat);
        return E;
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
        Float D     = 0.0;
        point_triangle_distance(P, T0, T1, T2, D);

        Vector12 GradD;
        point_triangle_distance_gradient(P, T0, T1, T2, GradD);

        Float dBdD;
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial E}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix12x12 HessD;
        point_triangle_distance_hessian(P, T0, T1, T2, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 E}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial E}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }

    __device__ Float EE_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& Ea0,
                                       const Vector3& Ea1,
                                       const Vector3& Eb0,
                                       const Vector3& Eb1)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);
        Float E = 0.0;
        sym::KappaBarrier(E, kappa, D, D_hat);
        return E;
    }


    __device__ void EE_barrier_gradient_hessian(Vector12&      G,
                                                Matrix12x12&   H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& Ea0,
                                                const Vector3& Ea1,
                                                const Vector3& Eb0,
                                                const Vector3& Eb1)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);

        Vector12 GradD;
        edge_edge_distance_gradient(Ea0, Ea1, Eb0, Eb1, GradD);

        Float dBdD;
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial E}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix12x12 HessD;
        edge_edge_distance_hessian(Ea0, Ea1, Eb0, Eb1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 E}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial E}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
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
        sym::KappaBarrier(E, kappa, D, D_hat);
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
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial E}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix9x9 HessD;
        point_edge_distance_hessian(P, E0, E1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 E}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial E}{\partial D} \frac{\partial^2 D}{\partial x^2}
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
        sym::KappaBarrier(E, kappa, D, D_hat);
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
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial E}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix6x6 HessD;
        point_point_distance_hessian(P0, P1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 E}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial E}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }
}  // namespace ipc_contact


void IPCSimplexContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle energy
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PT")
        .apply(PTs().size(),
               [PTs = PTs().viewer().name("PTs")] __device__(int i) {

               });
}

void IPCSimplexContact::do_assemble(ContactInfo& info) {}
}  // namespace uipc::backend::cuda