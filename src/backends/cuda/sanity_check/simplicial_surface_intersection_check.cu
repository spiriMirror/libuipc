#include <sanity_check/sanity_checker.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <collision_detection/info_stackless_bvh.h>
#include <backends/common/backend_path_tool.h>
#include <fstream>

namespace uipc::backend::cuda
{
namespace
{
// GPU-compatible triangle-edge intersection test (Moeller-Trumbore).
// Returns true if segment (E0, E1) intersects triangle (T0, T1, T2).
inline __device__ bool tri_edge_intersect_device(const Vector3& T0,
                                                  const Vector3& T1,
                                                  const Vector3& T2,
                                                  const Vector3& E0,
                                                  const Vector3& E1)
{
    constexpr Float eps = 1e-12;

    Vector3 edge = E1 - E0;
    Vector3 e1   = T1 - T0;
    Vector3 e2   = T2 - T0;
    Vector3 h    = edge.cross(e2);
    Float   a    = e1.dot(h);

    if(a > -eps && a < eps)
        return false;

    Float   f = 1.0 / a;
    Vector3 s = E0 - T0;
    Float   u = f * s.dot(h);
    if(u < 0.0 || u > 1.0)
        return false;

    Vector3 q = s.cross(e1);
    Float   v = f * edge.dot(q);
    if(v < 0.0 || u + v > 1.0)
        return false;

    Float t = f * e2.dot(q);
    return (t >= 0.0 && t <= 1.0);
}
}  // namespace

class SimplicialSurfaceIntersectionCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

    class Impl
    {
      public:
        SimSystemSlot<GlobalContactManager>           contact_manager;
        SimSystemSlot<GlobalVertexManager>             vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager>  surface_manager;

        InfoStacklessBVH              tri_bvh;
        InfoStacklessBVH::QueryBuffer ef_qbuffer;

        muda::DeviceBuffer<AABB>   tri_aabbs;
        muda::DeviceBuffer<IndexT> tri_bids;
        muda::DeviceBuffer<IndexT> tri_cids;

        muda::DeviceBuffer<AABB>   edge_aabbs;
        muda::DeviceBuffer<IndexT> edge_bids;
        muda::DeviceBuffer<IndexT> edge_cids;

        std::string_view ws;  // workspace path

        void check(CheckInfo& info);
        void export_intersected_mesh(span<const Vector2i> pairs,
                                     CheckInfo&           info);
    };

  protected:
    void do_build(BuildInfo& info) override
    {
        m_impl.contact_manager = require<GlobalContactManager>();
        m_impl.vertex_manager  = require<GlobalVertexManager>();
        m_impl.surface_manager = require<GlobalSimplicialSurfaceManager>();
        m_impl.ws              = workspace();
    }

    void do_init(InitInfo& info) override {}
    void do_check(CheckInfo& info) override { m_impl.check(info); }

  private:
    Impl m_impl;
};

void SimplicialSurfaceIntersectionCheck::Impl::check(CheckInfo& info)
{
    using namespace muda;

    auto Ps   = vertex_manager->positions();
    auto bids = vertex_manager->body_ids();
    auto cids = vertex_manager->contact_element_ids();

    auto Vs = surface_manager->surf_vertices();
    auto Es = surface_manager->surf_edges();
    auto Fs = surface_manager->surf_triangles();

    auto cmts = contact_manager->contact_mask_tabular();

    if(Es.size() == 0 || Fs.size() == 0)
        return;

    // Build triangle AABBs (tight, no expansion for intersection test)
    tri_aabbs.resize(Fs.size());
    tri_bids.resize(Fs.size());
    tri_cids.resize(Fs.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Fs.size(),
               [Fs   = Fs.viewer().name("Fs"),
                Vs   = Vs.viewer().name("Vs"),
                Ps   = Ps.cviewer().name("Ps"),
                bids = bids.cviewer().name("bids"),
                cids = cids.cviewer().name("cids"),
                aabbs = tri_aabbs.viewer().name("aabbs"),
                obids = tri_bids.viewer().name("obids"),
                ocids = tri_cids.viewer().name("ocids")] __device__(int i) mutable
               {
                   auto F = Fs(i);
                   AABB aabb;
                   aabb.extend(Ps(F[0]).cast<float>());
                   aabb.extend(Ps(F[1]).cast<float>());
                   aabb.extend(Ps(F[2]).cast<float>());
                   aabbs(i) = aabb;
                   obids(i) = bids(F[0]);
                   ocids(i) = cids(F[0]);
               });

    // Build edge AABBs (tight)
    edge_aabbs.resize(Es.size());
    edge_bids.resize(Es.size());
    edge_cids.resize(Es.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Es.size(),
               [Es   = Es.viewer().name("Es"),
                Vs   = Vs.viewer().name("Vs"),
                Ps   = Ps.cviewer().name("Ps"),
                bids = bids.cviewer().name("bids"),
                cids = cids.cviewer().name("cids"),
                aabbs = edge_aabbs.viewer().name("aabbs"),
                obids = edge_bids.viewer().name("obids"),
                ocids = edge_cids.viewer().name("ocids")] __device__(int i) mutable
               {
                   auto E = Es(i);
                   AABB aabb;
                   aabb.extend(Ps(E[0]).cast<float>());
                   aabb.extend(Ps(E[1]).cast<float>());
                   aabbs(i) = aabb;
                   obids(i) = bids(E[0]);
                   ocids(i) = cids(E[0]);
               });

    // Build BVH on triangles, query with edges.
    // LeafPred returns true for intersecting pairs → stored in QueryBuffer.
    tri_bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());

    tri_bvh.query(
        edge_aabbs.view(),
        edge_bids.view(),
        edge_cids.view(),
        cmts,
        [] __device__(InfoStacklessBVH::NodePredInfo) { return true; },
        [Vs = Vs.viewer().name("Vs"),
         Es = Es.viewer().name("Es"),
         Fs = Fs.viewer().name("Fs"),
         Ps = Ps.cviewer().name("Ps")] __device__(
            InfoStacklessBVH::LeafPredInfo leaf)
        {
            auto E = Es(leaf.i);
            auto F = Fs(leaf.j);

            // Skip if edge shares any vertex with triangle
            if(E[0] == F[0] || E[0] == F[1] || E[0] == F[2]
               || E[1] == F[0] || E[1] == F[1] || E[1] == F[2])
                return false;

            return tri_edge_intersect_device(
                Ps(F[0]), Ps(F[1]), Ps(F[2]), Ps(E[0]), Ps(E[1]));
        },
        ef_qbuffer);

    auto pair_count = ef_qbuffer.size();
    if(pair_count > 0)
    {
        logger::error("GPU SanityCheck: {} edge-triangle intersection(s) detected "
                      "(frame={}, newton={})",
                      pair_count,
                      info.frame(),
                      info.newton_iter());

        // Copy pairs to host and export
        std::vector<Vector2i> h_pairs(pair_count);
        ef_qbuffer.view().copy_to(h_pairs.data());
        export_intersected_mesh(h_pairs, info);
    }
}

void SimplicialSurfaceIntersectionCheck::Impl::export_intersected_mesh(
    span<const Vector2i> pairs,
    CheckInfo&           info)
{
    // 1. Copy surface data from GPU to host
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

    // 2. Mark intersected edges/triangles/vertices
    // pairs[k] = {edge_idx, tri_idx}
    std::vector<IndexT> vert_flag(h_Ps.size(), 0);
    std::vector<IndexT> edge_flag(h_Es.size(), 0);
    std::vector<IndexT> tri_flag(h_Fs.size(), 0);

    for(auto& p : pairs)
    {
        auto ei = p.x();
        auto fi = p.y();

        edge_flag[ei] = 1;
        tri_flag[fi]  = 1;

        auto E = h_Es[ei];
        vert_flag[h_Vs[E[0]]] = 1;
        vert_flag[h_Vs[E[1]]] = 1;

        auto F = h_Fs[fi];
        vert_flag[h_Vs[F[0]]] = 1;
        vert_flag[h_Vs[F[1]]] = 1;
        vert_flag[h_Vs[F[2]]] = 1;
    }

    // 3. Collect intersected vertices and build remap
    std::vector<IndexT> vert_remap(h_Ps.size(), static_cast<IndexT>(-1));
    std::vector<IndexT> active_verts;
    active_verts.reserve(pairs.size() * 5);
    for(SizeT i = 0; i < h_Ps.size(); ++i)
    {
        if(vert_flag[i])
        {
            vert_remap[i] = static_cast<IndexT>(active_verts.size());
            active_verts.push_back(i);
        }
    }

    // 4. Write .obj
    backend::BackendPathTool path_tool{ws};
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "sanity_check");
    auto file_path     = fmt::format("{}intersected.{}.{}.obj",
                                 output_folder.string(),
                                 info.frame(),
                                 info.newton_iter());

    std::ofstream file(file_path);
    if(!file.is_open())
    {
        logger::warn("GPU SanityCheck: failed to open {} for writing", file_path);
        return;
    }

    file << fmt::format("# GPU SanityCheck: {} intersecting edge-triangle pairs\n",
                        pairs.size());

    // Write vertices
    for(auto vi : active_verts)
    {
        auto& pos = h_Ps[vi];
        file << fmt::format("v {} {} {}\n", pos.x(), pos.y(), pos.z());
    }

    // Write intersected edges as lines
    for(SizeT i = 0; i < h_Es.size(); ++i)
    {
        if(!edge_flag[i])
            continue;
        auto E  = h_Es[i];
        auto v0 = vert_remap[h_Vs[E[0]]];
        auto v1 = vert_remap[h_Vs[E[1]]];
        // .obj is 1-indexed
        file << fmt::format("l {} {}\n", v0 + 1, v1 + 1);
    }

    // Write intersected triangles as faces
    for(SizeT i = 0; i < h_Fs.size(); ++i)
    {
        if(!tri_flag[i])
            continue;
        auto F  = h_Fs[i];
        auto v0 = vert_remap[h_Vs[F[0]]];
        auto v1 = vert_remap[h_Vs[F[1]]];
        auto v2 = vert_remap[h_Vs[F[2]]];
        file << fmt::format("f {} {} {}\n", v0 + 1, v1 + 1, v2 + 1);
    }

    logger::info("GPU SanityCheck: intersected mesh exported to {}", file_path);
}

REGISTER_SIM_SYSTEM(SimplicialSurfaceIntersectionCheck);
}  // namespace uipc::backend::cuda
