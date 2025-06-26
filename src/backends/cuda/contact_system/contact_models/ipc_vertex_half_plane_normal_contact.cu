#include <contact_system/vertex_half_plane_normal_contact.h>
#include <implicit_geometry/half_plane.h>
#include <contact_system/contact_models/ipc_vertex_half_plane_contact_function.h>
#include <kernel_cout.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda
{
class IPCVertexHalfPlaneNormalContact final : public VertexHalfPlaneNormalContact
{
  public:
    using VertexHalfPlaneNormalContact::VertexHalfPlaneNormalContact;

    virtual void do_build(BuildInfo& info) override
    {
        auto constitution =
            world().scene().info()["contact"]["constitution"].get<std::string>();
        if(constitution != "ipc")
        {
            throw SimSystemException("Constitution is not IPC");
        }

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

                       Float kt2 =
                           table(contact_ids(vI), half_plane_contact_ids(HI)).kappa
                           * dt * dt;

                       Float thickness = thicknesses(vI);

                       Es(I) = sym::ipc_vertex_half_contact::PH_barrier_energy(
                           kt2, d_hat, thickness, v, P, N);
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

                        Float kt2 =
                            table(contact_ids(vI), half_plane_contact_ids(HI)).kappa
                            * dt * dt;

                        Float thickness = thicknesses(vI);

                        Vector3   G;
                        Matrix3x3 H;

                        sym::ipc_vertex_half_contact::PH_barrier_gradient_hessian(
                            G, H, kt2, d_hat, thickness, v, P, N);

                        cuda::make_spd(H);

                        Grad(I).write(PH(0), G);
                        Hess(I).write(PH(0), PH(0), H);
                    });
        }
    }

    HalfPlane* half_plane = nullptr;
};

REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneNormalContact);
}  // namespace uipc::backend::cuda
