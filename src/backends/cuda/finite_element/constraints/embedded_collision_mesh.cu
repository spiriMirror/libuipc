// EmbeddedCollisionMesh — GPU backend
//
// Barycentric coupling: passive dense surface mesh driven by coarse FEM tet mesh.
//
//   Forward  (event_before_collision_detection): x_surface[i] = J * x_tet
//   Backward (event_after_contact_assembly):     grad_tet    += Jᵀ * grad_surface
//
// Components:
//   EmbeddedCollisionMeshVertexReporter  — registers surface verts in GlobalVertexManager (UID=3)
//   EmbeddedCollisionMeshSystem          — holds GPU buffers, drives forward/backward passes
//   EmbeddedCollisionMeshForceReporter   — injects tet_extra_grad into FEM linear system
//
// Reference: SOFA CudaBarycentricMapping — apply() / applyJT()

#include <global_geometry/vertex_reporter.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/fem_linear_subsystem_reporter.h>
#include <global_geometry/global_vertex_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <uipc/builtin/attribute_name.h>
#include <muda/muda.h>

namespace uipc::backend::cuda
{

// ================================================================
// Shared GPU buffers
// ================================================================
struct EmbeddedCollisionMeshData
{
    muda::DeviceBuffer<IndexT>   tet_index;           // [n_surface] — which tet each surface vert belongs to
    muda::DeviceBuffer<Vector4>  bary;                // [n_surface] — barycentric coords
    muda::DeviceBuffer<Vector4i> tet_cells;           // [n_tets]    — tet connectivity
    muda::DeviceBuffer<Vector3>  surface_pos_init;    // [n_surface] — initial surface positions (for reporter)
    muda::DeviceBuffer<Vector3>  surface_grad_dense;  // [n_surface] — scattered contact grads
    muda::DeviceBuffer<Vector3>  tet_extra_grad;      // [n_tet_verts] — accumulated grad for FEM

    IndexT tet_vertex_offset     = 0;
    IndexT surface_vertex_offset = 0;
    SizeT  n_tet_verts           = 0;
    SizeT  n_surface_verts       = 0;
    SizeT  n_tets                = 0;
};

// ================================================================
// CUDA kernels — file-scope to satisfy nvcc __device__ lambda rules
// ================================================================

__global__ void kernel_ecm_forward(
    const Vector3*  tet_pos,
    const Vector4i* tet_cells,
    const IndexT*   tet_index,
    const Vector4*  bary,
    Vector3*        surface_pos,
    int             n_surface)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n_surface)
        return;

    Vector4i cell   = tet_cells[tet_index[i]];
    surface_pos[i]  = bary[i](0) * tet_pos[cell[0]]
                    + bary[i](1) * tet_pos[cell[1]]
                    + bary[i](2) * tet_pos[cell[2]]
                    + bary[i](3) * tet_pos[cell[3]];
}

// atomicAdd on Float* flat — Vector3 layout not guaranteed contiguous for CUDA atomics
__global__ void kernel_ecm_backward(
    const Vector3*  surface_grad,
    const Vector4i* tet_cells,
    const IndexT*   tet_index,
    const Vector4*  bary,
    Float*          tet_extra_grad_flat,
    int             n_surface)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n_surface)
        return;

    Vector3  f    = surface_grad[i];
    Vector4i cell = tet_cells[tet_index[i]];
    for(int j = 0; j < 4; ++j)
    {
        Float w = bary[i](j);
        int   v = cell[j];
        atomicAdd(&tet_extra_grad_flat[v * 3 + 0], w * f(0));
        atomicAdd(&tet_extra_grad_flat[v * 3 + 1], w * f(1));
        atomicAdd(&tet_extra_grad_flat[v * 3 + 2], w * f(2));
    }
}

// Scatter CDoubletVectorView<Float,3> COO entries → dense surface_grad_dense
__global__ void kernel_coo_to_dense(
    const int*   coo_indices,
    const Float* coo_values_flat,  // stride 3 (reinterpret of Segment<Float,3>[])
    int          coo_count,
    IndexT       surface_off,
    int          n_surface,
    Float*       dense_flat)       // stride 3 (reinterpret of Vector3[])
{
    int I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= coo_count)
        return;

    int l_i = coo_indices[I] - (int)surface_off;
    if(l_i < 0 || l_i >= n_surface)
        return;

    atomicAdd(&dense_flat[l_i * 3 + 0], coo_values_flat[I * 3 + 0]);
    atomicAdd(&dense_flat[l_i * 3 + 1], coo_values_flat[I * 3 + 1]);
    atomicAdd(&dense_flat[l_i * 3 + 2], coo_values_flat[I * 3 + 2]);
}

// ================================================================
// Forward declarations
// ================================================================
class EmbeddedCollisionMeshSystem;
class EmbeddedCollisionMeshForceReporter;

// ================================================================
// EmbeddedCollisionMeshVertexReporter
//
// Registers the passive surface mesh in GlobalVertexManager.
// UID=3 — placed after ABD(0), FEM(1), HalfPlane(2) in the
// sorted reporter list, so surface verts are appended at the end.
//
// report_displacements() → zero: the surface is driven by the
// forward pass kernel writing directly into GlobalVertexManager
// positions, not by the dx system. CCD still sees the surface
// because AABB is built from [pos, pos+dx*alpha]; with dx=0 the
// AABB degenerates to a point but is still d_hat-expanded and
// inserted in the BVH — no silent skip.
// ================================================================
class EmbeddedCollisionMeshVertexReporter final : public VertexReporter
{
  public:
    using VertexReporter::VertexReporter;

    EmbeddedCollisionMeshSystem* m_ecm_system = nullptr;

    void do_build(BuildInfo&) override;

    void do_report_count(VertexCountInfo& info) override;

    void do_report_attributes(VertexAttributeInfo& info) override;

    void do_report_displacements(VertexDisplacementInfo& info) override
    {
        // Surface verts are repositioned each Newton iter by kernel_ecm_forward
        // writing directly into the GlobalVertexManager positions buffer.
        // The dx system is bypassed — displacement is always zero here.
        info.displacements().fill(Vector3::Zero());
    }

    U64 get_uid() const noexcept override { return 3; }
};

// ================================================================
// EmbeddedCollisionMeshSystem
//
// Owns GPU buffers and drives the two passes.
// All methods that access SimSystem API (world(), require<T>())
// are instance methods of this class, which inherits SimSystem.
// ================================================================
class EmbeddedCollisionMeshSystem final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    EmbeddedCollisionMeshData data;

    // CPU-side initial positions — needed by the vertex reporter at frame 0
    std::vector<Vector3> h_surface_pos_init;

    // Called by EmbeddedCollisionMeshForceReporter::do_assemble()
    void assemble(FEMLinearSubsystemReporter::AssembleInfo& info)
    {
        auto& d = data;
        if(d.n_tet_verts == 0)
            return;

        muda::ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                d.n_tet_verts,
                [grads = info.gradients().viewer().name("ecm_grad"),
                 extra = d.tet_extra_grad.cviewer().name("ecm_extra"),
                 t_off = (int)d.tet_vertex_offset] __device__(int i) mutable
                {
                    Vector3 g = extra(i);
                    if(g.squaredNorm() > Float(0))
                        grads(i).write(t_off + i, g);
                });
    }

  protected:
    void do_build() override
    {
        on_init_scene([this]() { _init(*this); });
        on_before_collision_detection([this]() { _forward(*this); });
        on_after_contact_assembly([this]() { _scatter_surface_grad(*this); });
    }

  private:
    static void _init(EmbeddedCollisionMeshSystem& self)
    {
        auto& d        = self.data;
        auto  geo_slots = self.world().scene().geometries();

        const geometry::SimplicialComplex* surface_sc = nullptr;
        const geometry::SimplicialComplex* tet_sc     = nullptr;

        for(SizeT gi = 0; gi < geo_slots.size(); ++gi)
        {
            auto* sc =
                geo_slots[gi]->geometry().as<geometry::SimplicialComplex>();
            if(!sc || !sc->meta().find<IndexT>("ecm_driven"))
                continue;

            surface_sc = sc;
            auto tet_id_attr = sc->meta().find<IndexT>("ecm_tet_geo_id");
            if(!tet_id_attr)
                break;

            IndexT tet_gi = tet_id_attr->view()[0];
            if(tet_gi >= 0 && tet_gi < (IndexT)geo_slots.size())
                tet_sc = geo_slots[tet_gi]
                             ->geometry()
                             .as<geometry::SimplicialComplex>();
            break;
        }

        if(!surface_sc || !tet_sc)
            return;

        _init_buffers(self, *surface_sc, *tet_sc);
    }

    static void _init_buffers(EmbeddedCollisionMeshSystem&       self,
                               const geometry::SimplicialComplex& surface_sc,
                               const geometry::SimplicialComplex& tet_sc)
    {
        auto& d = self.data;

        auto idx_attr  = surface_sc.vertices().find<IndexT>("ecm_tet_index");
        auto bary_attr = surface_sc.vertices().find<Vector4>("ecm_bary");
        if(!idx_attr || !bary_attr)
            return;

        auto idx_view  = idx_attr->view();
        auto bary_view = bary_attr->view();
        d.n_surface_verts = idx_view.size();

        d.tet_index.resize(d.n_surface_verts);
        d.tet_index.view().copy_from(idx_view.data());

        d.bary.resize(d.n_surface_verts);
        d.bary.view().copy_from(bary_view.data());

        // global_vertex_offset is written on ALL geometries by their vertex reporters
        // (both FEM and passive). It is available here because _init runs in
        // on_init_scene, which fires after all reporters have initialized.
        auto tet_off_slot =
            tet_sc.meta().find<IndexT>(builtin::global_vertex_offset);
        if(!tet_off_slot)
            return;
        d.tet_vertex_offset = tet_off_slot->view()[0];
        d.n_tet_verts       = tet_sc.positions().view().size();

        auto surf_off_slot =
            surface_sc.meta().find<IndexT>(builtin::global_vertex_offset);
        if(!surf_off_slot)
            return;
        d.surface_vertex_offset = surf_off_slot->view()[0];

        auto& fem    = self.require<FiniteElementMethod>();
        auto  tets_v = fem.tets();
        d.n_tets = tets_v.size();
        d.tet_cells.resize(d.n_tets);
        d.tet_cells.view().copy_from(tets_v);

        d.surface_grad_dense.resize(d.n_surface_verts, Vector3::Zero());
        d.tet_extra_grad.resize(d.n_tet_verts, Vector3::Zero());
    }

    static void _forward(EmbeddedCollisionMeshSystem& self)
    {
        auto& d = self.data;
        if(d.n_surface_verts == 0)
            return;

        auto* gvm    = &self.require<GlobalVertexManager>();
        auto  all_pos = gvm->positions();

        // Write directly into the GlobalVertexManager positions buffer at the
        // surface mesh offset — this is what the CCD and contact systems see.
        Vector3*       surf_ptr = const_cast<Vector3*>(all_pos.data()) + d.surface_vertex_offset;
        const Vector3* tet_ptr  = all_pos.data() + d.tet_vertex_offset;

        int block = 256;
        int grid  = ((int)d.n_surface_verts + block - 1) / block;
        kernel_ecm_forward<<<grid, block>>>(
            tet_ptr,
            d.tet_cells.data(),
            d.tet_index.data(),
            d.bary.data(),
            surf_ptr,
            (int)d.n_surface_verts);
    }

    static void _scatter_surface_grad(EmbeddedCollisionMeshSystem& self)
    {
        auto& d = self.data;
        if(d.n_surface_verts == 0)
            return;

        auto* dytopo = &self.require<GlobalDyTopoEffectManager>();
        auto  coo    = dytopo->gradients();

        d.surface_grad_dense.view().fill(Vector3::Zero());
        d.tet_extra_grad.view().fill(Vector3::Zero());

        int coo_count = (int)coo.doublet_count();
        if(coo_count == 0)
            return;

        {
            int block = 256;
            int grid  = (coo_count + block - 1) / block;
            kernel_coo_to_dense<<<grid, block>>>(
                coo.indices().data(),
                reinterpret_cast<const Float*>(coo.values().data()),
                coo_count,
                (IndexT)d.surface_vertex_offset,
                (int)d.n_surface_verts,
                reinterpret_cast<Float*>(d.surface_grad_dense.data()));
        }

        {
            int block = 256;
            int grid  = ((int)d.n_surface_verts + block - 1) / block;
            kernel_ecm_backward<<<grid, block>>>(
                d.surface_grad_dense.data(),
                d.tet_cells.data(),
                d.tet_index.data(),
                d.bary.data(),
                reinterpret_cast<Float*>(d.tet_extra_grad.data()),
                (int)d.n_surface_verts);
        }
    }
};

REGISTER_SIM_SYSTEM(EmbeddedCollisionMeshSystem);

// ================================================================
// EmbeddedCollisionMeshVertexReporter — method bodies
// (defined after EmbeddedCollisionMeshSystem is complete)
// ================================================================

void EmbeddedCollisionMeshVertexReporter::do_build(BuildInfo&)
{
    m_ecm_system = &require<EmbeddedCollisionMeshSystem>();
    // VertexReporter::do_build() (the non-virtual base) calls add_reporter(this)
    // automatically — no need to call it here.
}

void EmbeddedCollisionMeshVertexReporter::do_report_count(VertexCountInfo& info)
{
    info.count(m_ecm_system->data.n_surface_verts);
}

void EmbeddedCollisionMeshVertexReporter::do_report_attributes(VertexAttributeInfo& info)
{
    auto& d = m_ecm_system->data;

    if(info.frame() == 0)
    {
        // Write initial positions from CPU buffer
        auto& h_pos = m_ecm_system->h_surface_pos_init;
        if(!h_pos.empty())
            info.positions().copy_from(h_pos.data());

        // The surface mesh has no contact element — use default (0)
        // Thickness from d_hat default; no special body_id needed

        // Write global_vertex_offset on the surface geometry meta attribute
        // so EmbeddedCollisionMeshSystem::_init_buffers() can read it.
        auto global_offset = info.coindices().offset();
        auto geo_slots     = world().scene().geometries();
        for(SizeT gi = 0; gi < geo_slots.size(); ++gi)
        {
            auto* sc = geo_slots[gi]->geometry().as<geometry::SimplicialComplex>();
            if(!sc || !sc->meta().find<IndexT>("ecm_driven"))
                continue;

            auto gvo = sc->meta().find<IndexT>(builtin::global_vertex_offset);
            if(!gvo)
                gvo = sc->meta().create<IndexT>(builtin::global_vertex_offset);
            view(*gvo)[0] = (IndexT)global_offset;
            break;
        }
    }
    // Frames > 0: positions are updated by kernel_ecm_forward directly into the
    // GlobalVertexManager buffer — no copy needed here.
}

REGISTER_SIM_SYSTEM(EmbeddedCollisionMeshVertexReporter);

// ================================================================
// EmbeddedCollisionMeshForceReporter
// ================================================================
class EmbeddedCollisionMeshForceReporter final
    : public FEMLinearSubsystemReporter
{
  public:
    using FEMLinearSubsystemReporter::FEMLinearSubsystemReporter;

    SimSystemSlot<EmbeddedCollisionMeshSystem> m_ecm_system;

    void do_build(BuildInfo& info) override
    {
        m_ecm_system = require<EmbeddedCollisionMeshSystem>();
    }

    void do_init(InitInfo& info) override {}

    void do_report_extent(ReportExtentInfo& info) override
    {
        info.gradient_count(m_ecm_system->data.n_tet_verts);
        info.hessian_count(0);
    }

    void do_assemble(AssembleInfo& info) override
    {
        m_ecm_system->assemble(info);
    }
};

REGISTER_SIM_SYSTEM(EmbeddedCollisionMeshForceReporter);

}  // namespace uipc::backend::cuda
