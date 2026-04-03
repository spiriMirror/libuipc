#include <sanity_check/sanity_checker.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <implicit_geometry/half_plane.h>
#include <backends/common/backend_path_tool.h>
#include <fstream>

namespace uipc::backend::cuda
{
class HalfPlaneVertexDistanceCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

    class Impl
    {
      public:
        SimSystemSlot<GlobalContactManager>           contact_manager;
        SimSystemSlot<GlobalVertexManager>             vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager>  surface_manager;
        SimSystemSlot<HalfPlane>                       half_plane;

        muda::DeviceVar<IndexT> violation_count;

        std::string_view ws;

        void check(CheckInfo& info);
        void export_close_mesh(span<const IndexT> h_vert_flags, CheckInfo& info);
    };

  protected:
    void do_build(BuildInfo& info) override
    {
        m_impl.contact_manager = require<GlobalContactManager>();
        m_impl.vertex_manager  = require<GlobalVertexManager>();
        m_impl.surface_manager = require<GlobalSimplicialSurfaceManager>();
        m_impl.half_plane      = require<HalfPlane>();
        m_impl.ws              = workspace();
    }

    void do_init(InitInfo& info) override {}
    void do_check(CheckInfo& info) override { m_impl.check(info); }

  private:
    Impl m_impl;
};

void HalfPlaneVertexDistanceCheck::Impl::check(CheckInfo& info)
{
    using namespace muda;

    auto hp_normals   = half_plane->normals();
    auto hp_positions = half_plane->positions();

    if(hp_normals.size() == 0)
        return;

    auto Vs    = surface_manager->surf_vertices();
    auto Ps    = vertex_manager->positions();
    auto thick = vertex_manager->thicknesses();

    if(Vs.size() == 0)
        return;

    auto cmts = contact_manager->contact_mask_tabular();

    // For each half-plane instance, check all surface vertices.
    // Half-planes are few (usually 1-3), so we launch a kernel per vertex
    // for each half-plane. The mask check uses host-side half-plane contact/subscene IDs.
    auto hp_geo_infos = half_plane->geo_infos();

    // Accumulate per-vertex violation flags on GPU
    DeviceBuffer<IndexT> vert_flags(Vs.size());
    vert_flags.fill(0);

    violation_count = 0;

    // For each half-plane instance, run a parallel check over all vertices
    SizeT hp_count = hp_normals.size();
    for(SizeT hi = 0; hi < hp_count; ++hi)
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(Vs.size(),
                   [Vs    = Vs.viewer().name("Vs"),
                    Ps    = Ps.cviewer().name("Ps"),
                    thick = thick.cviewer().name("thick"),
                    hp_N  = hp_normals.viewer().name("hp_N"),
                    hp_P  = hp_positions.viewer().name("hp_P"),
                    flags = vert_flags.viewer().name("flags"),
                    violation = violation_count.viewer().name("violation"),
                    hi] __device__(int i) mutable
                   {
                       auto vI = Vs(i);

                       const Vector3& N = hp_N(hi);
                       const Vector3& P = hp_P(hi);
                       const Vector3& V = Ps(vI);
                       Float thickness  = thick(vI);

                       // signed distance = (V - P) · N - thickness
                       Float d = (V - P).dot(N) - thickness;

                       if(d <= 0.0)
                       {
                           flags(i) = 1;
                           atomicAdd(violation.data(), IndexT(1));
                       }
                   });
    }

    IndexT h_violation = violation_count;
    if(h_violation > 0)
    {
        logger::error("GPU SanityCheck: {} half-plane vertex distance violation(s) "
                      "detected (frame={}, newton={})",
                      h_violation,
                      info.frame(),
                      info.newton_iter());

        // Export close mesh
        std::vector<IndexT> h_vert_flags(Vs.size());
        vert_flags.view().copy_to(h_vert_flags.data());
        export_close_mesh(h_vert_flags, info);
    }
}

void HalfPlaneVertexDistanceCheck::Impl::export_close_mesh(
    span<const IndexT> h_vert_flags,
    CheckInfo&         info)
{
    auto Vs_dev = surface_manager->surf_vertices();
    auto Es_dev = surface_manager->surf_edges();
    auto Fs_dev = surface_manager->surf_triangles();
    auto Ps_dev = vertex_manager->positions();

    std::vector<IndexT>   h_Vs(Vs_dev.size());
    std::vector<Vector2i> h_Es(Es_dev.size());
    std::vector<Vector3i> h_Fs(Fs_dev.size());
    std::vector<Vector3>  h_Ps(Ps_dev.size());

    Vs_dev.copy_to(h_Vs.data());
    Es_dev.copy_to(h_Es.data());
    Fs_dev.copy_to(h_Fs.data());
    Ps_dev.copy_to(h_Ps.data());

    // Build remap: only include flagged vertices
    // h_vert_flags is indexed by surface vertex index, but flags global vertex
    // Actually it's indexed by surf_vertex LOCAL index
    std::vector<IndexT> global_vert_flag(h_Ps.size(), 0);
    for(SizeT i = 0; i < h_Vs.size(); ++i)
    {
        if(h_vert_flags[i])
            global_vert_flag[h_Vs[i]] = 1;
    }

    std::vector<IndexT> vert_remap(h_Ps.size(), static_cast<IndexT>(-1));
    std::vector<IndexT> active_verts;
    for(SizeT i = 0; i < h_Ps.size(); ++i)
    {
        if(global_vert_flag[i])
        {
            vert_remap[i] = static_cast<IndexT>(active_verts.size());
            active_verts.push_back(i);
        }
    }

    backend::BackendPathTool path_tool{ws};
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "sanity_check");
    auto file_path     = fmt::format("{}halfplane_close.{}.{}.obj",
                                 output_folder.string(),
                                 info.frame(),
                                 info.newton_iter());

    std::ofstream file(file_path);
    if(!file.is_open())
    {
        logger::warn("GPU SanityCheck: failed to open {} for writing", file_path);
        return;
    }

    file << fmt::format("# GPU SanityCheck: half-plane vertex distance violations\n");

    for(auto vi : active_verts)
    {
        auto& pos = h_Ps[vi];
        file << fmt::format("v {} {} {}\n", pos.x(), pos.y(), pos.z());
    }

    // Write edges where both vertices are flagged
    for(SizeT i = 0; i < h_Es.size(); ++i)
    {
        auto E  = h_Es[i];
        auto v0 = h_Vs[E[0]];
        auto v1 = h_Vs[E[1]];
        if(global_vert_flag[v0] && global_vert_flag[v1])
            file << fmt::format("l {} {}\n", vert_remap[v0] + 1, vert_remap[v1] + 1);
    }

    // Write triangles where all vertices are flagged
    for(SizeT i = 0; i < h_Fs.size(); ++i)
    {
        auto F  = h_Fs[i];
        auto v0 = h_Vs[F[0]];
        auto v1 = h_Vs[F[1]];
        auto v2 = h_Vs[F[2]];
        if(global_vert_flag[v0] && global_vert_flag[v1] && global_vert_flag[v2])
            file << fmt::format("f {} {} {}\n",
                                vert_remap[v0] + 1,
                                vert_remap[v1] + 1,
                                vert_remap[v2] + 1);
    }

    logger::info("GPU SanityCheck: half-plane close mesh exported to {}", file_path);
}

REGISTER_SIM_SYSTEM(HalfPlaneVertexDistanceCheck);
}  // namespace uipc::backend::cuda
