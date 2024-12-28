#include <contact_system/vertex_half_plane_normal_contact.h>
#include <implicit_geometry/half_plane.h>
#include <contact_system/contact_models/ipc_vertex_half_plane_contact_function.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
class IPCVertexHalfPlaneNormalContact final : public VertexHalfPlaneNormalContact
{
  public:
    using VertexHalfPlaneNormalContact::VertexHalfPlaneNormalContact;

    virtual void do_build(BuildInfo& info) override
    {
        half_plane = &require<HalfPlane>();
    }

    virtual void do_compute_energy(EnergyInfo& info)
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.PHs().size(),
                   [Es  = info.energies().viewer().name("Es"),
                    PHs = info.PHs().viewer().name("PHs"),
                    plane_positions = half_plane->positions().viewer().name("plane_positions"),
                    plane_normals = half_plane->normals().viewer().name("plane_normals"),
                    table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    half_plane_contact_ids = half_plane->contact_ids().viewer().name("half_plane_contact_ids"),
                    Ps = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
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

                       Float kappa =
                           table(contact_ids(vI), half_plane_contact_ids(HI)).kappa;

                       Float thickness = thicknesses(vI);

                       Es(I) = sym::ipc_vertex_half_contact::PH_barrier_energy(
                           kappa * dt * dt, d_hat, thickness, v, P, N);
                   });
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace muda;

        if(info.PHs().size())
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(
                    info.PHs().size(),
                    [Grad = info.gradients().viewer().name("Grad"),
                     Hess = info.hessians().viewer().name("Hess"),
                     PHs  = info.PHs().viewer().name("PHs"),
                     plane_positions = half_plane->positions().viewer().name("plane_positions"),
                     plane_normals = half_plane->normals().viewer().name("plane_normals"),
                     table = info.contact_tabular().viewer().name("contact_tabular"),
                     contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                     half_plane_contact_ids = half_plane->contact_ids().viewer().name("half_plane_contact_ids"),
                     Ps = info.positions().viewer().name("Ps"),
                     thicknesses = info.thicknesses().viewer().name("thicknesses"),
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

                        Float kappa =
                            table(contact_ids(vI), half_plane_contact_ids(HI)).kappa;

                        Float thickness = thicknesses(vI);

                        sym::ipc_vertex_half_contact::PH_barrier_gradient_hessian(
                            Grad(I), Hess(I), kappa * dt * dt, d_hat, thickness, v, P, N);

                        //cout << "Grad: " << Grad(I).transpose().eval() << "\n";
                        //cout << "Hess: " << Hess(I).eval() << "\n";
                    });
        }
    }

    HalfPlane* half_plane = nullptr;
};

REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneNormalContact);
}  // namespace uipc::backend::cuda
