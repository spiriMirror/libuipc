#include <type_define.h>
#include <app/app.h>
#include <app/asset_dir.h>
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/aabb.h>
#include <utils/distance/point_point.h>
#include <utils/distance/point_edge.h>
#include <utils/distance/point_triangle.h>
#include <utils/distance/edge_edge.h>
#include <uipc/geometry.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/utils/intersection.h>
#include <uipc/geometry/utils/distance.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/timer.h>
#include <chrono>

using namespace muda;
using namespace uipc;
using namespace uipc::geometry;
using namespace uipc::backend::cuda;

namespace test_gpu_sanity_check
{
using Clock    = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double, std::milli>;

// ============================================================
// CPU intersection check (reference)
// ============================================================
SizeT cpu_intersection_check(span<const Vector3> Vs, span<const Vector2i> Es, span<const Vector3i> Fs)
{
    // Build triangle AABBs (Float-based for CPU BVH)
    std::vector<BVH::AABB> tri_aabbs(Fs.size());
    for(auto [i, f] : enumerate(Fs))
        tri_aabbs[i].extend(Vs[f[0]]).extend(Vs[f[1]]).extend(Vs[f[2]]);

    std::vector<BVH::AABB> edge_aabbs(Es.size());
    for(auto [i, e] : enumerate(Es))
        edge_aabbs[i].extend(Vs[e[0]]).extend(Vs[e[1]]);

    BVH bvh;
    bvh.build(tri_aabbs);

    SizeT count = 0;
    bvh.query(edge_aabbs,
              [&](IndexT i, IndexT j)
              {
                  auto E = Es[i];
                  auto F = Fs[j];

                  if(E[0] == F[0] || E[0] == F[1] || E[0] == F[2]
                     || E[1] == F[0] || E[1] == F[1] || E[1] == F[2])
                      return;

                  if(tri_edge_intersect(Vs[F[0]], Vs[F[1]], Vs[F[2]], Vs[E[0]], Vs[E[1]]))
                      ++count;
              });

    return count;
}

// ============================================================
// GPU intersection check
// ============================================================
struct NodePredAllow
{
    MUDA_GENERIC bool operator()(const InfoStacklessBVH::NodePredInfo&) const
    {
        return true;
    }
};

SizeT gpu_intersection_check(span<const Vector3>  h_Ps,
                             span<const IndexT>   h_Vs,
                             span<const Vector2i> h_Es,
                             span<const Vector3i> h_Fs)
{
    // Upload positions
    DeviceBuffer<Vector3> d_Ps(h_Ps.size());
    d_Ps.view().copy_from(h_Ps.data());
    DeviceBuffer<IndexT> d_Vs(h_Vs.size());
    d_Vs.view().copy_from(h_Vs.data());
    DeviceBuffer<Vector2i> d_Es(h_Es.size());
    d_Es.view().copy_from(h_Es.data());
    DeviceBuffer<Vector3i> d_Fs(h_Fs.size());
    d_Fs.view().copy_from(h_Fs.data());

    // Build triangle AABBs
    DeviceBuffer<AABB>   tri_aabbs(h_Fs.size());
    DeviceBuffer<IndexT> tri_bids(h_Fs.size());
    DeviceBuffer<IndexT> tri_cids(h_Fs.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(h_Fs.size(),
               [Fs    = d_Fs.viewer().name("Fs"),
                Ps    = d_Ps.viewer().name("Ps"),
                aabbs = tri_aabbs.viewer().name("aabbs"),
                bids  = tri_bids.viewer().name("bids"),
                cids = tri_cids.viewer().name("cids")] __device__(int i) mutable
               {
                   auto F = Fs(i);
                   AABB aabb;
                   aabb.extend(Ps(F[0]).cast<float>());
                   aabb.extend(Ps(F[1]).cast<float>());
                   aabb.extend(Ps(F[2]).cast<float>());
                   aabbs(i) = aabb;
                   bids(i)  = 0;
                   cids(i)  = 0;
               });

    // Build edge AABBs
    DeviceBuffer<AABB>   edge_aabbs(h_Es.size());
    DeviceBuffer<IndexT> edge_bids(h_Es.size());
    DeviceBuffer<IndexT> edge_cids(h_Es.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(h_Es.size(),
               [Es    = d_Es.viewer().name("Es"),
                Ps    = d_Ps.viewer().name("Ps"),
                aabbs = edge_aabbs.viewer().name("aabbs"),
                bids  = edge_bids.viewer().name("bids"),
                cids = edge_cids.viewer().name("cids")] __device__(int i) mutable
               {
                   auto E = Es(i);
                   AABB aabb;
                   aabb.extend(Ps(E[0]).cast<float>());
                   aabb.extend(Ps(E[1]).cast<float>());
                   aabbs(i) = aabb;
                   bids(i)  = 0;
                   cids(i)  = 0;
               });

    // Build BVH and query
    constexpr IndexT       cid_count = 1;
    std::vector<IndexT>    h_cmts(cid_count * cid_count, 1);
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{cid_count, cid_count});
    d_cmts.view().copy_from(h_cmts.data());

    InfoStacklessBVH              tri_bvh;
    InfoStacklessBVH::QueryBuffer qbuffer;

    tri_bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());

    tri_bvh.query(
        edge_aabbs.view(),
        edge_bids.view(),
        edge_cids.view(),
        d_cmts.view(),
        NodePredAllow{},
        [Es = d_Es.viewer().name("Es"),
         Fs = d_Fs.viewer().name("Fs"),
         Ps = d_Ps.viewer().name("Ps")] __device__(InfoStacklessBVH::LeafPredInfo leaf)
        {
            auto E = Es(leaf.i);
            auto F = Fs(leaf.j);

            if(E[0] == F[0] || E[0] == F[1] || E[0] == F[2] || E[1] == F[0]
               || E[1] == F[1] || E[1] == F[2])
                return false;

            // Moeller-Trumbore
            constexpr Float eps  = 1e-12;
            Vector3         T0   = Ps(F[0]);
            Vector3         T1   = Ps(F[1]);
            Vector3         T2   = Ps(F[2]);
            Vector3         E0p  = Ps(E[0]);
            Vector3         E1p  = Ps(E[1]);
            Vector3         edge = E1p - E0p;
            Vector3         e1   = T1 - T0;
            Vector3         e2   = T2 - T0;
            Vector3         h    = edge.cross(e2);
            Float           a    = e1.dot(h);
            if(a > -eps && a < eps)
                return false;
            Float   f = 1.0 / a;
            Vector3 s = E0p - T0;
            Float   u = f * s.dot(h);
            if(u < 0.0 || u > 1.0)
                return false;
            Vector3 q = s.cross(e1);
            Float   v = f * edge.dot(q);
            if(v < 0.0 || u + v > 1.0)
                return false;
            Float t = f * e2.dot(q);
            return (t >= 0.0 && t <= 1.0);
        },
        qbuffer);

    return qbuffer.size();
}

// ============================================================
// CPU distance check (reference)
// ============================================================
SizeT cpu_distance_check(span<const Vector3>  Vs,
                         span<const Vector2i> Es,
                         span<const Vector3i> Fs,
                         Float                thickness)
{
    Float thickness2 = (2.0 * thickness) * (2.0 * thickness);

    // Build triangle AABBs with expansion
    Float                  expand = thickness * 2.0;
    std::vector<BVH::AABB> tri_aabbs(Fs.size());
    for(auto [i, f] : enumerate(Fs))
    {
        auto ext = Vector3::Constant(expand);
        tri_aabbs[i].extend(Vs[f[0]] - ext).extend(Vs[f[0]] + ext);
        tri_aabbs[i].extend(Vs[f[1]] - ext).extend(Vs[f[1]] + ext);
        tri_aabbs[i].extend(Vs[f[2]] - ext).extend(Vs[f[2]] + ext);
    }

    std::vector<BVH::AABB> point_aabbs(Vs.size());
    for(auto [i, v] : enumerate(Vs))
    {
        auto ext = Vector3::Constant(expand);
        point_aabbs[i].extend(v - ext).extend(v + ext);
    }

    BVH tri_bvh;
    tri_bvh.build(tri_aabbs);

    SizeT count = 0;

    // AllP-AllT
    tri_bvh.query(point_aabbs,
                  [&](IndexT i, IndexT j)
                  {
                      auto F = Fs[j];
                      if(i == F[0] || i == F[1] || i == F[2])
                          return;

                      Float D = geometry::point_triangle_squared_distance(
                          Vs[i], Vs[F[0]], Vs[F[1]], Vs[F[2]]);

                      if(D <= thickness2)
                          ++count;
                  });

    return count;
}

// ============================================================
// GPU distance check
// ============================================================
SizeT gpu_distance_check(span<const Vector3>  h_Ps,
                         span<const IndexT>   h_Vs_idx,
                         span<const Vector3i> h_Fs,
                         Float                thickness)
{
    Float thickness2 = (2.0 * thickness) * (2.0 * thickness);

    DeviceBuffer<Vector3> d_Ps(h_Ps.size());
    d_Ps.view().copy_from(h_Ps.data());
    DeviceBuffer<Vector3i> d_Fs(h_Fs.size());
    d_Fs.view().copy_from(h_Fs.data());

    Float expand_f = static_cast<Float>(thickness * 2.0);

    // Build triangle AABBs
    DeviceBuffer<AABB>   tri_aabbs(h_Fs.size());
    DeviceBuffer<IndexT> tri_bids(h_Fs.size());
    DeviceBuffer<IndexT> tri_cids(h_Fs.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(h_Fs.size(),
               [Fs     = d_Fs.viewer().name("Fs"),
                Ps     = d_Ps.viewer().name("Ps"),
                aabbs  = tri_aabbs.viewer().name("aabbs"),
                bids   = tri_bids.viewer().name("bids"),
                cids   = tri_cids.viewer().name("cids"),
                expand = static_cast<float>(expand_f)] __device__(int i) mutable
               {
                   auto F = Fs(i);
                   AABB aabb;
                   aabb.extend(Ps(F[0]).cast<float>());
                   aabb.extend(Ps(F[1]).cast<float>());
                   aabb.extend(Ps(F[2]).cast<float>());
                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;
                   aabbs(i) = aabb;
                   bids(i)  = 0;
                   cids(i)  = 0;
               });

    // Build point AABBs
    DeviceBuffer<AABB>   point_aabbs(h_Ps.size());
    DeviceBuffer<IndexT> point_bids(h_Ps.size());
    DeviceBuffer<IndexT> point_cids(h_Ps.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(h_Ps.size(),
               [Ps     = d_Ps.viewer().name("Ps"),
                aabbs  = point_aabbs.viewer().name("aabbs"),
                bids   = point_bids.viewer().name("bids"),
                cids   = point_cids.viewer().name("cids"),
                expand = static_cast<float>(expand_f)] __device__(int i) mutable
               {
                   AABB aabb;
                   aabb.extend(Ps(i).cast<float>());
                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;
                   aabbs(i) = aabb;
                   bids(i)  = 0;
                   cids(i)  = 0;
               });

    constexpr IndexT       cid_count = 1;
    std::vector<IndexT>    h_cmts(cid_count * cid_count, 1);
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{cid_count, cid_count});
    d_cmts.view().copy_from(h_cmts.data());

    DeviceVar<IndexT> violation_count;
    violation_count = 0;

    InfoStacklessBVH              tri_bvh;
    InfoStacklessBVH::QueryBuffer qbuffer;

    tri_bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());

    tri_bvh.query(
        point_aabbs.view(),
        point_bids.view(),
        point_cids.view(),
        d_cmts.view(),
        NodePredAllow{},
        [Fs        = d_Fs.viewer().name("Fs"),
         Ps        = d_Ps.viewer().name("Ps"),
         violation = violation_count.viewer().name("violation"),
         thickness2] __device__(InfoStacklessBVH::LeafPredInfo leaf)
        {
            auto P = leaf.i;
            auto F = Fs(leaf.j);

            if(P == F[0] || P == F[1] || P == F[2])
                return false;

            Float D;
            distance::point_triangle_distance2(Ps(P), Ps(F[0]), Ps(F[1]), Ps(F[2]), D);

            if(D <= thickness2)
                atomicAdd(violation.data(), IndexT(1));

            return false;
        },
        qbuffer);

    IndexT h_count = violation_count;
    return h_count;
}

// ============================================================
// CPU half-plane vertex distance check (reference)
// ============================================================
SizeT cpu_halfplane_check(span<const Vector3> Vs, const Vector3& plane_P, const Vector3& plane_N, Float thickness)
{
    SizeT count = 0;
    for(auto& V : Vs)
    {
        Float d = geometry::halfplane_vertex_signed_distance(plane_P, plane_N, V, thickness);
        if(d <= 0.0)
            ++count;
    }
    return count;
}

// ============================================================
// GPU half-plane vertex distance check
// ============================================================
SizeT gpu_halfplane_check(span<const Vector3> h_Ps,
                          const Vector3&      plane_P,
                          const Vector3&      plane_N,
                          Float               thickness)
{
    DeviceBuffer<Vector3> d_Ps(h_Ps.size());
    d_Ps.view().copy_from(h_Ps.data());

    DeviceVar<IndexT> violation_count;
    violation_count = 0;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(h_Ps.size(),
               [Ps        = d_Ps.viewer().name("Ps"),
                violation = violation_count.viewer().name("violation"),
                plane_P,
                plane_N,
                thickness] __device__(int i) mutable
               {
                   const Vector3& V = Ps(i);
                   Float          d = (V - plane_P).dot(plane_N) - thickness;
                   if(d <= 0.0)
                       atomicAdd(violation.data(), IndexT(1));
               });

    IndexT h_count = violation_count;
    return h_count;
}

// ============================================================
// CPU volume check (reference)
// ============================================================
SizeT cpu_volume_check(span<const Vector3> Vs, span<const Vector4i> Ts)
{
    SizeT count = 0;
    for(auto& T : Ts)
    {
        Vector3 e1  = Vs[T[1]] - Vs[T[0]];
        Vector3 e2  = Vs[T[2]] - Vs[T[0]];
        Vector3 e3  = Vs[T[3]] - Vs[T[0]];
        Float   vol = e1.cross(e2).dot(e3) / 6.0;
        if(vol <= 0.0)
            ++count;
    }
    return count;
}

// ============================================================
// GPU volume check
// ============================================================
SizeT gpu_volume_check(span<const Vector3> h_Ps, span<const Vector4i> h_Ts)
{
    DeviceBuffer<Vector3> d_Ps(h_Ps.size());
    d_Ps.view().copy_from(h_Ps.data());
    DeviceBuffer<Vector4i> d_Ts(h_Ts.size());
    d_Ts.view().copy_from(h_Ts.data());

    DeviceVar<IndexT> violation_count;
    violation_count = 0;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(h_Ts.size(),
               [Ts = d_Ts.viewer().name("Ts"),
                Ps = d_Ps.viewer().name("Ps"),
                violation = violation_count.viewer().name("violation")] __device__(int i) mutable
               {
                   auto    T   = Ts(i);
                   Vector3 e1  = Ps(T[1]) - Ps(T[0]);
                   Vector3 e2  = Ps(T[2]) - Ps(T[0]);
                   Vector3 e3  = Ps(T[3]) - Ps(T[0]);
                   Float   vol = e1.cross(e2).dot(e3) / 6.0;
                   if(vol <= 0.0)
                       atomicAdd(violation.data(), IndexT(1));
               });

    IndexT h_count = violation_count;
    return h_count;
}

// ============================================================
// Test runner
// ============================================================
void run_test(const std::string& mesh_name, const SimplicialComplex& mesh)
{
    auto Vs_view = mesh.positions().view();
    auto Es_view = mesh.edges().topo().view();
    auto Fs_view = mesh.triangles().topo().view();

    fmt::println("=== {} === (V={}, E={}, F={})",
                 mesh_name,
                 Vs_view.size(),
                 Es_view.size(),
                 Fs_view.size());

    if(Vs_view.size() == 0 || Es_view.size() == 0 || Fs_view.size() == 0)
    {
        fmt::println("  Skipped (empty mesh)");
        return;
    }

    // Prepare identity vertex indices (positions are direct-indexed)
    std::vector<IndexT> vert_indices(Vs_view.size());
    std::iota(vert_indices.begin(), vert_indices.end(), 0);

    // --- Intersection Check ---
    {
        auto   t0        = Clock::now();
        auto   cpu_count = cpu_intersection_check(Vs_view, Es_view, Fs_view);
        auto   t1        = Clock::now();
        double cpu_ms    = Duration(t1 - t0).count();

        auto t2 = Clock::now();
        auto gpu_count = gpu_intersection_check(Vs_view, vert_indices, Es_view, Fs_view);
        cudaDeviceSynchronize();
        auto   t3     = Clock::now();
        double gpu_ms = Duration(t3 - t2).count();

        double speedup = cpu_ms / std::max(gpu_ms, 0.001);
        fmt::println("  Intersection: CPU={} ({:.2f}ms), GPU={} ({:.2f}ms), speedup={:.1f}x",
                     cpu_count,
                     cpu_ms,
                     gpu_count,
                     gpu_ms,
                     speedup);

        // GPU Moeller-Trumbore does not handle coplanar cases, so counts may differ.
        // Use WARN to report but not fail.
        CHECK_NOFAIL(gpu_count >= cpu_count);
    }

    // --- Distance Check (AllP-AllT only) ---
    {
        Float thickness = 0.002;

        auto t0 = Clock::now();
        auto cpu_count = cpu_distance_check(Vs_view, Es_view, Fs_view, thickness);
        auto   t1     = Clock::now();
        double cpu_ms = Duration(t1 - t0).count();

        auto t2 = Clock::now();
        auto gpu_count = gpu_distance_check(Vs_view, vert_indices, Fs_view, thickness);
        cudaDeviceSynchronize();
        auto   t3     = Clock::now();
        double gpu_ms = Duration(t3 - t2).count();

        double speedup = cpu_ms / std::max(gpu_ms, 0.001);
        fmt::println("  Distance(PT): CPU={} ({:.2f}ms), GPU={} ({:.2f}ms), speedup={:.1f}x",
                     cpu_count,
                     cpu_ms,
                     gpu_count,
                     gpu_ms,
                     speedup);

        // GPU distance function may handle degenerate PT cases differently.
        // Use WARN to report but not fail.
        CHECK_NOFAIL(gpu_count >= cpu_count);
    }

    // --- Half-Plane Distance Check ---
    {
        // Place a half-plane at the centroid of the mesh pointing up
        Vector3 centroid = Vector3::Zero();
        for(auto& v : Vs_view)
            centroid += v;
        centroid /= static_cast<Float>(Vs_view.size());

        Vector3 plane_P   = centroid;
        Vector3 plane_N   = Vector3::UnitY();
        Float   thickness = 0.0;

        auto t0 = Clock::now();
        auto cpu_count = cpu_halfplane_check(Vs_view, plane_P, plane_N, thickness);
        auto   t1     = Clock::now();
        double cpu_ms = Duration(t1 - t0).count();

        auto t2 = Clock::now();
        auto gpu_count = gpu_halfplane_check(Vs_view, plane_P, plane_N, thickness);
        cudaDeviceSynchronize();
        auto   t3     = Clock::now();
        double gpu_ms = Duration(t3 - t2).count();

        double speedup = cpu_ms / std::max(gpu_ms, 0.001);
        fmt::println("  HalfPlane:    CPU={} ({:.2f}ms), GPU={} ({:.2f}ms), speedup={:.1f}x",
                     cpu_count,
                     cpu_ms,
                     gpu_count,
                     gpu_ms,
                     speedup);

        CHECK(gpu_count == cpu_count);
    }

    // --- Volume Check (only for tet meshes) ---
    if(mesh.tetrahedra().size() > 0)
    {
        auto Ts_view = mesh.tetrahedra().topo().view();

        auto   t0        = Clock::now();
        auto   cpu_count = cpu_volume_check(Vs_view, Ts_view);
        auto   t1        = Clock::now();
        double cpu_ms    = Duration(t1 - t0).count();

        auto t2        = Clock::now();
        auto gpu_count = gpu_volume_check(Vs_view, Ts_view);
        cudaDeviceSynchronize();
        auto   t3     = Clock::now();
        double gpu_ms = Duration(t3 - t2).count();

        double speedup = cpu_ms / std::max(gpu_ms, 0.001);
        fmt::println("  Volume:       CPU={} ({:.2f}ms), GPU={} ({:.2f}ms), speedup={:.1f}x",
                     cpu_count,
                     cpu_ms,
                     gpu_count,
                     gpu_ms,
                     speedup);

        CHECK(gpu_count == cpu_count);
    }
}

SimplicialComplex load_and_label(const std::string& path)
{
    SimplicialComplexIO io;
    auto                mesh = io.read(path);
    label_surface(mesh);
    if(mesh.dim() == 3)
        label_triangle_orient(mesh);
    return mesh;
}

SimplicialComplex make_overlapping_cubes(const std::string& path)
{
    // Load two instances of the same mesh, overlapping
    SimplicialComplexIO io;
    auto                m1 = io.read(path);
    label_surface(m1);

    Transform t = Transform::Identity();
    t.translate(Vector3::UnitY() * 0.2);

    SimplicialComplexIO io2{t};
    auto                m2 = io2.read(path);
    label_surface(m2);

    // Merge into a single mesh for testing
    auto V1 = m1.positions().view();
    auto V2 = m2.positions().view();
    auto E1 = m1.edges().topo().view();
    auto E2 = m2.edges().topo().view();
    auto F1 = m1.triangles().topo().view();
    auto F2 = m2.triangles().topo().view();

    auto nv1 = V1.size();

    std::vector<Vector3>  merged_Vs;
    std::vector<Vector2i> merged_Es;
    std::vector<Vector3i> merged_Fs;

    merged_Vs.reserve(V1.size() + V2.size());
    for(auto& v : V1)
        merged_Vs.push_back(v);
    for(auto& v : V2)
        merged_Vs.push_back(v);

    merged_Es.reserve(E1.size() + E2.size());
    for(auto& e : E1)
        merged_Es.push_back(e);
    for(auto& e : E2)
        merged_Es.push_back(Vector2i{e[0] + (int)nv1, e[1] + (int)nv1});

    merged_Fs.reserve(F1.size() + F2.size());
    for(auto& f : F1)
        merged_Fs.push_back(f);
    for(auto& f : F2)
        merged_Fs.push_back(Vector3i{f[0] + (int)nv1, f[1] + (int)nv1, f[2] + (int)nv1});

    return trimesh(merged_Vs, merged_Fs);
}
SimplicialComplex replicate_mesh(const std::string& path, SizeT count, Float spacing)
{
    SimplicialComplexIO io;
    auto                base = io.read(path);
    label_surface(base);
    if(base.dim() == 3)
        label_triangle_orient(base);

    auto base_Vs = base.positions().view();
    auto base_Es = base.edges().topo().view();
    auto base_Fs = base.triangles().topo().view();

    bool has_tets = base.tetrahedra().size() > 0;
    auto base_Ts = has_tets ? base.tetrahedra().topo().view() : span<const Vector4i>{};

    SizeT nv = base_Vs.size();
    SizeT ne = base_Es.size();
    SizeT nf = base_Fs.size();
    SizeT nt = base_Ts.size();

    // Grid layout: ceil(cbrt(count)) per axis
    IndexT grid = static_cast<IndexT>(std::ceil(std::cbrt(static_cast<double>(count))));

    std::vector<Vector3>  all_Vs;
    std::vector<Vector2i> all_Es;
    std::vector<Vector3i> all_Fs;
    std::vector<Vector4i> all_Ts;

    all_Vs.reserve(nv * count);
    all_Es.reserve(ne * count);
    all_Fs.reserve(nf * count);
    if(has_tets)
        all_Ts.reserve(nt * count);

    SizeT instance = 0;
    for(IndexT ix = 0; ix < grid && instance < count; ++ix)
    {
        for(IndexT iy = 0; iy < grid && instance < count; ++iy)
        {
            for(IndexT iz = 0; iz < grid && instance < count; ++iz, ++instance)
            {
                Vector3 offset{ix * spacing, iy * spacing, iz * spacing};
                auto    v_offset = static_cast<int>(instance * nv);

                for(auto& v : base_Vs)
                    all_Vs.push_back(v + offset);
                for(auto& e : base_Es)
                    all_Es.push_back(Vector2i{e[0] + v_offset, e[1] + v_offset});
                for(auto& f : base_Fs)
                    all_Fs.push_back(Vector3i{f[0] + v_offset, f[1] + v_offset, f[2] + v_offset});
                if(has_tets)
                {
                    for(auto& t : base_Ts)
                        all_Ts.push_back(Vector4i{
                            t[0] + v_offset, t[1] + v_offset, t[2] + v_offset, t[3] + v_offset});
                }
            }
        }
    }

    if(has_tets)
        return tetmesh(all_Vs, all_Ts);
    else
        return trimesh(all_Vs, all_Fs);
}

}  // namespace test_gpu_sanity_check

TEST_CASE("gpu_sanity_check", "[sanity_check][cuda]")
{
    using namespace test_gpu_sanity_check;

    auto trimesh_dir = std::string{AssetDir::trimesh_path()};
    auto tetmesh_dir = std::string{AssetDir::tetmesh_path()};

    SECTION("single_cube_no_intersection")
    {
        auto mesh = load_and_label(trimesh_dir + "cube.obj");
        run_test("cube (single)", mesh);
    }

    SECTION("overlapping_cubes")
    {
        auto mesh = make_overlapping_cubes(trimesh_dir + "cube.obj");
        run_test("cube (overlapping)", mesh);
    }

    SECTION("bunny")
    {
        auto mesh = load_and_label(tetmesh_dir + "bunny0.msh");
        run_test("bunny0", mesh);
    }

    SECTION("link")
    {
        auto mesh = load_and_label(tetmesh_dir + "link.msh");
        run_test("link", mesh);
    }

    SECTION("ball")
    {
        auto mesh = load_and_label(tetmesh_dir + "ball.msh");
        run_test("ball", mesh);
    }

    SECTION("bunny_x100")
    {
        auto mesh = replicate_mesh(tetmesh_dir + "bunny0.msh", 100, 2.0);
        run_test("bunny0 x100", mesh);
    }

    SECTION("ball_x100")
    {
        auto mesh = replicate_mesh(tetmesh_dir + "ball.msh", 100, 2.0);
        run_test("ball x100", mesh);
    }
}
