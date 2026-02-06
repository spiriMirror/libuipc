#include <contact_system/vertex_half_plane_frictional_contact.h>
#include <implicit_geometry/half_plane.h>
#include <contact_system/contact_models/ipc_vertex_half_plane_contact_function.h>
#include <kernel_cout.h>
#include <collision_detection/global_trajectory_filter.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
class IPCVertexHalfPlaneFrictionalContact final : public VertexHalfPlaneFrictionalContact
{
  public:
    using VertexHalfPlaneFrictionalContact::VertexHalfPlaneFrictionalContact;

    virtual void do_build(BuildInfo& info) override
    {
        auto constitution =
            world().scene().config().find<std::string>("contact/constitution");
        if(constitution->view()[0] != "ipc")
        {
            throw SimSystemException("Constitution is not IPC");
        }

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
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v  = info.eps_velocity(),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    half_plane_vertex_offset = info.half_plane_vertex_offset(),
                    dt = info.dt()] __device__(int I) mutable
                   {
                       Vector2i PH = PHs(I);

                       IndexT vI = PH(0);
                       IndexT HI = PH(1);

                       Vector3 v      = Ps(vI);
                       Vector3 prev_v = prev_Ps(vI);
                       Vector3 P      = plane_positions(HI);
                       Vector3 N      = plane_normals(HI);

                       Float d_hat = d_hats(vI);

                       ContactCoeff coeff =
                           table(contact_ids(vI), contact_ids(HI + half_plane_vertex_offset));
                       Float kt2 = coeff.kappa * dt * dt;
                       Float mu  = coeff.mu;

                       Float thickness = thicknesses(vI);

                       Es(I) = PH_friction_energy(
                           kt2, d_hat, thickness, mu, eps_v * dt, prev_v, v, P, N);
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
                .apply(
                    info.friction_PHs().size(),
                    [gradient_only = info.gradient_only(),
                     Grad          = info.gradients().viewer().name("Grad"),
                     Hess          = info.hessians().viewer().name("Hess"),
                     PHs           = info.friction_PHs().viewer().name("PHs"),
                     plane_positions = half_plane->positions().viewer().name("plane_positions"),
                     plane_normals = half_plane->normals().viewer().name("plane_normals"),
                     table = info.contact_tabular().viewer().name("contact_tabular"),
                     contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                     Ps      = info.positions().viewer().name("Ps"),
                     prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                     thicknesses = info.thicknesses().viewer().name("thicknesses"),
                     eps_v  = info.eps_velocity(),
                     d_hats = info.d_hats().viewer().name("d_hats"),
                     half_plane_vertex_offset = info.half_plane_vertex_offset(),
                     dt = info.dt()] __device__(int I) mutable
                    {
                        Vector2i PH = PHs(I);

                        IndexT vI = PH(0);
                        IndexT HI = PH(1);

                        Vector3 v      = Ps(vI);
                        Vector3 prev_v = prev_Ps(vI);
                        Vector3 P      = plane_positions(HI);
                        Vector3 N      = plane_normals(HI);

                        Float d_hat = d_hats(vI);

                        ContactCoeff coeff =
                            table(contact_ids(vI), contact_ids(HI + half_plane_vertex_offset));
                        Float kt2 = coeff.kappa * dt * dt;
                        Float mu  = coeff.mu;

                        Float thickness = thicknesses(vI);

                        Vector3 G;
                        if(gradient_only)
                        {
                            PH_friction_gradient(
                                G, kt2, d_hat, thickness, mu, eps_v * dt, prev_v, v, P, N);
                            Grad(I).write(vI, G);
                        }
                        else
                        {
                            Matrix3x3 H;
                            PH_friction_gradient_hessian(
                                G, H, kt2, d_hat, thickness, mu, eps_v * dt, prev_v, v, P, N);
                            cuda::make_spd(H);
                            Grad(I).write(vI, G);
                            Hess(I).write(vI, vI, H);
                        }
                    });
        }
    }

    HalfPlane* half_plane = nullptr;
};

REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneFrictionalContact);
}  // namespace uipc::backend::cuda
