#include <contact_system/vertex_half_plane_frictional_contact.h>
#include <implicit_geometry/half_plane.h>
#include <contact_system/contact_models/ipc_vertex_half_plane_contact_function.h>
#include <kernel_cout.h>
#include <collision_detection/global_trajectory_filter.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <utils/make_spd.h>
#include <pipeline/ipc_pipeline_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void do_compute_energy_kernel(cuda_tool::BufferView<Float> Es,
                                             cuda_tool::CBufferView<Vector2i> PHs,
                                             cuda_tool::CBufferView<Vector3> plane_positions,
                                             cuda_tool::CBufferView<Vector3> plane_normals,
                                             cuda_tool::CDense2D<ContactCoeff> table,
                                             cuda_tool::CBufferView<IndexT> contact_ids,
                                             cuda_tool::CBufferView<Vector3> Ps,
                                             cuda_tool::CBufferView<Vector3> prev_Ps,
                                             cuda_tool::CBufferView<Float> thicknesses,
                                             Float eps_v,
                                             cuda_tool::CBufferView<Float> d_hats,
                                             IndexT half_plane_vertex_offset,
                                             Float dt,
                                             int   n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;

        using namespace sym::ipc_vertex_half_contact;

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
    }

    __global__ void do_assemble_kernel(bool gradient_only,
                                       cuda_tool::DoubletVectorView<Float, 3> Grad,
                                       cuda_tool::TripletMatrixView<Float, 3> Hess,
                                       cuda_tool::CBufferView<Vector2i> PHs,
                                       cuda_tool::CBufferView<Vector3> plane_positions,
                                       cuda_tool::CBufferView<Vector3> plane_normals,
                                       cuda_tool::CDense2D<ContactCoeff> table,
                                       cuda_tool::CBufferView<IndexT> contact_ids,
                                       cuda_tool::CBufferView<Vector3> Ps,
                                       cuda_tool::CBufferView<Vector3> prev_Ps,
                                       cuda_tool::CBufferView<Float> thicknesses,
                                       Float eps_v,
                                       cuda_tool::CBufferView<Float> d_hats,
                                       IndexT half_plane_vertex_offset,
                                       Float dt,
                                       int   n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;

        using namespace sym::ipc_vertex_half_contact;

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
    }
}  // namespace

class IPCVertexHalfPlaneFrictionalContact final : public VertexHalfPlaneFrictionalContact
{
  public:
    using VertexHalfPlaneFrictionalContact::VertexHalfPlaneFrictionalContact;

    virtual void do_build(BuildInfo& info) override
    {
        require<IPCPipelineFlag>();
        half_plane = &require<HalfPlane>();
    }

    virtual void do_compute_energy(EnergyInfo& info)
    {
        using namespace cuda_tool;
        using namespace sym::ipc_vertex_half_contact;

        if(info.friction_PHs().size() > 0)
            do_compute_energy_kernel<<<cuda_tool::best_grid_dim((int)info.friction_PHs().size(),
                                                                do_compute_energy_kernel),
                                       cuda_tool::best_block_dim(do_compute_energy_kernel),
                                       0,
                                       nullptr>>>(info.energies().viewer(),
                                                  info.friction_PHs().viewer(),
                                                  half_plane->positions().viewer(),
                                                  half_plane->normals().viewer(),
                                                  info.contact_tabular().viewer(),
                                                  info.contact_element_ids().viewer(),
                                                  info.positions().viewer(),
                                                  info.prev_positions().viewer(),
                                                  info.thicknesses().viewer(),
                                                  info.eps_velocity(),
                                                  info.d_hats().viewer(),
                                                  info.half_plane_vertex_offset(),
                                                  info.dt(),
                                                  (int)info.friction_PHs().size());
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace cuda_tool;
        using namespace sym::ipc_vertex_half_contact;

        if(info.friction_PHs().size())
        {
            do_assemble_kernel<<<cuda_tool::best_grid_dim((int)info.friction_PHs().size(),
                                                          do_assemble_kernel),
                                 cuda_tool::best_block_dim(do_assemble_kernel),
                                 0,
                                 nullptr>>>(info.gradient_only(),
                                            info.gradients().viewer(),
                                            info.hessians().viewer(),
                                            info.friction_PHs().viewer(),
                                            half_plane->positions().viewer(),
                                            half_plane->normals().viewer(),
                                            info.contact_tabular().viewer(),
                                            info.contact_element_ids().viewer(),
                                            info.positions().viewer(),
                                            info.prev_positions().viewer(),
                                            info.thicknesses().viewer(),
                                            info.eps_velocity(),
                                            info.d_hats().viewer(),
                                            info.half_plane_vertex_offset(),
                                            info.dt(),
                                            (int)info.friction_PHs().size());
        }
    }

    HalfPlane* half_plane = nullptr;
};

REGISTER_SIM_SYSTEM(IPCVertexHalfPlaneFrictionalContact);
}  // namespace uipc::backend::cuda
