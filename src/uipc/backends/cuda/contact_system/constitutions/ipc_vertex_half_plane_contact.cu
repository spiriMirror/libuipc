#include <contact_system/constitutions/ipc_vertex_half_plane_contact.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneContact);

void IPCVertexHalfPlaneContact::do_build(BuildInfo& info)
{
    half_plane = &require<HalfPlane>();
}

namespace sym::ipc_vertex_half_contact
{
#include "sym/ipc_contact.inl"
#include "sym/vertex_half_plane_distance.inl"

    __device__ Float PH_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& v,
                                       const Vector3& P,
                                       const Vector3& N)
    {
        Float D_hat = squared_d_hat;
        Float D;
        HalfPlaneD(D, v, P, N);

        MUDA_ASSERT(D < D_hat, "D=%f,D_hat=%f, why?", D, D_hat);

        Float E = 0.0;
        KappaBarrier(E, kappa, D, D_hat);

        return E;
    }

    __device__ void PH_barrier_gradient_hessian(Vector3&       G,
                                                Matrix3x3&     H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& v,
                                                const Vector3& P,
                                                const Vector3& N)
    {
        Float D_hat = squared_d_hat;
        Float D;
        HalfPlaneD(D, v, P, N);

        MUDA_ASSERT(D < D_hat, "D=%f,D_hat=%f, why?", D, D_hat);

        Float dBdD = 0.0;
        dKappaBarrierdD(dBdD, kappa, D, D_hat);

        Vector3 dDdx;
        dHalfPlaneDdx(dDdx, v, P, N);

        G = dBdD * dDdx;

        Float ddBddD = 0.0;
        ddKappaBarrierddD(ddBddD, kappa, D, D_hat);

        Matrix3x3 ddDddx;
        ddHalfPlaneDddx(ddDddx, v, P, N);

        cout << "ddDddx:\n" << ddDddx << "\n";

        H = ddBddD * dDdx * dDdx.transpose() + dBdD * ddDddx;
    }

}  // namespace sym::ipc_vertex_half_contact

void IPCVertexHalfPlaneContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PHs().size(),
               [Es  = info.energies().viewer().name("Es"),
                PHs = info.PHs().viewer().name("PHs"),
                plane_positions = half_plane->positions().viewer().name("plane_positions"),
                plane_normals = half_plane->normals().viewer().name("plane_normals"),
                table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                Ps = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int I) mutable
               {
                   Vector2i PH = PHs(I);

                   IndexT vI = PH(0);
                   IndexT HI = PH(1);

                   Vector3 v = Ps(vI);
                   Vector3 P = plane_positions(HI);
                   Vector3 N = plane_normals(HI);

                   Float kappa = table(contact_ids(vI), contact_ids(HI)).kappa;

                   Float D_hat = d_hat * d_hat;


                   Es(I) = sym::ipc_vertex_half_contact::PH_barrier_energy(
                       kappa * dt * dt, D_hat, v, P, N);
               });
}

void IPCVertexHalfPlaneContact::do_assemble(ContactInfo& info)
{
    using namespace muda;

    if(info.PHs().size())
    {
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.PHs().size(),
                   [Grad = info.gradients().viewer().name("Grad"),
                    Hess = info.hessians().viewer().name("Hess"),
                    PHs  = info.PHs().viewer().name("PHs"),
                    plane_positions = half_plane->positions().viewer().name("plane_positions"),
                    plane_normals = half_plane->normals().viewer().name("plane_normals"),
                    table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    Ps = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int I) mutable
                   {
                       Vector2i PH = PHs(I);

                       IndexT vI = PH(0);
                       IndexT HI = PH(1);

                       Vector3 v = Ps(vI);
                       Vector3 P = plane_positions(HI);
                       Vector3 N = plane_normals(HI);

                       Float kappa = table(contact_ids(vI), contact_ids(HI)).kappa;

                       Float D_hat = d_hat * d_hat;

                       sym::ipc_vertex_half_contact::PH_barrier_gradient_hessian(
                           Grad(I), Hess(I), kappa * dt * dt, D_hat, v, P, N);

                       cout << "PH:" << PH.transpose().eval() << "\n"
                            << Grad(I) << "\n";
                   });

        int a = 1;
    }
}
}  // namespace uipc::backend::cuda
