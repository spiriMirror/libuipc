#include <contact_system/contact_models/ipc_vertex_half_plane_normal_contact.h>
#include <contact_system/contact_models/ipc_vertex_half_contact_function.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneNormalContact);

void IPCVertexHalfPlaneNormalContact::do_build(BuildInfo& info)
{
    half_plane = &require<HalfPlane>();
}

void IPCVertexHalfPlaneNormalContact::do_compute_energy(EnergyInfo& info)
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

                   Float kappa = table(contact_ids(vI), 0).kappa;

                   Float D_hat = d_hat * d_hat;

                   Es(I) = sym::ipc_vertex_half_contact::PH_barrier_energy(
                       kappa * dt * dt, D_hat, v, P, N);

                   //Es(I) = sym::ipc_vertex_half_contact::GIPC_ground_energy(
                   //    kappa * dt * dt, D_hat, v, P, N);
               });
}

void IPCVertexHalfPlaneNormalContact::do_assemble(ContactInfo& info)
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

                       Float kappa = table(contact_ids(vI), 0).kappa;

                       Float D_hat = d_hat * d_hat;

                       sym::ipc_vertex_half_contact::PH_barrier_gradient_hessian(
                           Grad(I), Hess(I), kappa * dt * dt, D_hat, v, P, N);

                       //sym::ipc_vertex_half_contact::GIPC_ground_gradient_hessian(
                       //    Grad(I), Hess(I), kappa * dt * dt, D_hat, v, P, N);
                   });

        int a = 1;
    }
}
}  // namespace uipc::backend::cuda
