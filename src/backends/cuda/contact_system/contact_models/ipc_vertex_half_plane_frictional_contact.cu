#include <contact_system/vertex_half_plane_frictional_contact.h>
#include <implicit_geometry/half_plane.h>
#include <contact_system/contact_models/ipc_vertex_half_plane_contact_function.h>
#include <kernel_cout.h>
#include <collision_detection/global_trajectory_filter.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>

namespace uipc::backend::cuda
{
class IPCVertexHalfPlaneFrictionalContact final : public VertexHalfPlaneFrictionalContact
{
  public:
    using VertexHalfPlaneFrictionalContact::VertexHalfPlaneFrictionalContact;

    virtual void do_build(BuildInfo& info) override
    {
        half_plane = &require<HalfPlane>();
    }

    virtual void do_compute_energy(EnergyInfo& info)
    {
        using namespace muda;
        using namespace sym::ipc_vertex_half_contact;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.friction_PHs().size(),
                   [Es  = info.energies().viewer().name("Es"),
                    PHs = info.friction_PHs().viewer().name("PHs"),
                    plane_positions = half_plane->positions().viewer().name("plane_positions"),
                    plane_normals = half_plane->normals().viewer().name("plane_normals"),
                    table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    half_plane_contact_ids = half_plane->contact_ids().viewer().name("half_plane_contact_ids"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int I) mutable
                   {
                       Vector2i PH = PHs(I);

                       IndexT vI = PH(0);
                       IndexT HI = PH(1);

                       Vector3 v      = Ps(vI);
                       Vector3 prev_v = prev_Ps(vI);
                       Vector3 P      = plane_positions(HI);
                       Vector3 N      = plane_normals(HI);

                       ContactCoeff coeff =
                           table(contact_ids(vI), half_plane_contact_ids(HI));
                       Float kappa = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       Float D_hat = d_hat * d_hat;

                       Float thickness = thicknesses(vI);

                       Es(I) = PH_friction_energy(
                           kappa, d_hat, thickness, mu, eps_v * dt, prev_v, v, P, N);
                   });
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace muda;
        using namespace sym::ipc_vertex_half_contact;

        if(info.friction_PHs().size())
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(info.friction_PHs().size(),
                       [Grad = info.gradients().viewer().name("Grad"),
                        Hess = info.hessians().viewer().name("Hess"),
                        PHs  = info.friction_PHs().viewer().name("PHs"),
                        plane_positions = half_plane->positions().viewer().name("plane_positions"),
                        plane_normals = half_plane->normals().viewer().name("plane_normals"),
                        table = info.contact_tabular().viewer().name("contact_tabular"),
                        contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                        half_plane_contact_ids =
                            half_plane->contact_ids().viewer().name("half_plane_contact_ids"),
                        Ps = info.positions().viewer().name("Ps"),
                        prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                        thicknesses = info.thicknesses().viewer().name("thicknesses"),
                        eps_v = info.eps_velocity(),
                        d_hat = info.d_hat(),
                        dt    = info.dt()] __device__(int I) mutable
                       {
                           Vector2i PH = PHs(I);

                           IndexT vI = PH(0);
                           IndexT HI = PH(1);

                           Vector3 v      = Ps(vI);
                           Vector3 prev_v = prev_Ps(vI);
                           Vector3 P      = plane_positions(HI);
                           Vector3 N      = plane_normals(HI);

                           ContactCoeff coeff =
                               table(contact_ids(vI), half_plane_contact_ids(HI));
                           Float kappa = coeff.kappa;
                           Float mu    = coeff.mu;

                           Float thickness = thicknesses(vI);

                           Float     dt2 = dt * dt;
                           Vector3   G;
                           Matrix3x3 H;

                           PH_friction_gradient_hessian(
                               G, H, kappa, d_hat, thickness, mu, eps_v * dt, prev_v, v, P, N);

                           Grad(I) = G * dt2;
                           Hess(I) = H * dt2;
                       });
        }
    }

    HalfPlane* half_plane = nullptr;
};

REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneFrictionalContact);
}  // namespace uipc::backend::cuda
