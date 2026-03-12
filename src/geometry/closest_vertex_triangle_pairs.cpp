#include <uipc/geometry/utils/closest_vertex_triangle_pairs.h>
#include <uipc/geometry/utils/distance.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/attribute_friend.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/vector.h>
#include <ranges>
#include <string_view>
#include <optional>

namespace uipc::geometry
{
namespace
{
    using AABB = BVH::AABB;

    AABB triangle_aabb(span<const Vector3> positions, const Vector3i& tri)
    {
        AABB box;
        box.extend(positions[tri(0)]);
        box.extend(positions[tri(1)]);
        box.extend(positions[tri(2)]);
        return box;
    }

    AABB point_aabb(const Vector3& p, Float half_extent)
    {
        AABB box;
        box.extend(p - Vector3::Constant(half_extent));
        box.extend(p + Vector3::Constant(half_extent));
        return box;
    }
}  // namespace

Geometry closest_vertex_triangle_pairs(const SimplicialComplex& vertex_mesh,
                                       const SimplicialComplex& triangle_mesh,
                                       Float                    max_distance,
                                       std::string_view max_distance_attr,
                                       std::string_view group)
{
    const auto  V_pos    = vertex_mesh.positions().view();
    const auto  tri_topo = triangle_mesh.triangles().topo().view();
    const auto  T_pos    = triangle_mesh.positions().view();
    const SizeT nV       = V_pos.size();
    const SizeT nTri     = tri_topo.size();

    std::optional<span<const Float>> per_vertex_max_d;
    if(!max_distance_attr.empty())
    {
        if(auto slot = vertex_mesh.vertices().find<Float>(max_distance_attr))
            per_vertex_max_d = slot->view();
    }

    std::optional<span<const IndexT>> vert_group;
    if(!group.empty())
    {
        if(auto slot = vertex_mesh.vertices().find<IndexT>(group))
            vert_group = slot->view();
    }

    std::optional<span<const IndexT>> tri_group;
    if(!group.empty())
    {
        if(auto slot = triangle_mesh.triangles().find<IndexT>(group))
            tri_group = slot->view();
    }

    vector<IndexT> compact_to_orig_tri;
    compact_to_orig_tri.reserve(nTri);
    for(IndexT i : std::views::iota(0, static_cast<IndexT>(nTri)))
    {
        if(tri_group && (*tri_group)[i] != 1)
            continue;
        compact_to_orig_tri.push_back(i);
    }

    vector<AABB> tri_aabbs;
    tri_aabbs.reserve(compact_to_orig_tri.size());
    std::ranges::transform(compact_to_orig_tri,
                           std::back_inserter(tri_aabbs),
                           [&](IndexT i)
                           { return triangle_aabb(T_pos, tri_topo[i]); });

    if(tri_aabbs.empty())
    {
        Geometry out;
        out.instances().resize(0);
        out.instances().create<Vector2i>(builtin::topo);
        return out;
    }

    BVH bvh;
    bvh.build(tri_aabbs);

    vector<IndexT> vert_orig;
    vector<AABB>   query_aabbs;
    vert_orig.reserve(nV);
    query_aabbs.reserve(nV);
    for(IndexT v : std::views::iota(0, static_cast<IndexT>(nV)))
    {
        if(vert_group && (*vert_group)[v] != 1)
            continue;
        Float max_d = per_vertex_max_d ? (*per_vertex_max_d)[v] : max_distance;
        if(max_d <= 0)
            continue;
        vert_orig.push_back(v);
        query_aabbs.push_back(point_aabb(V_pos[v], max_d));
    }

    if(query_aabbs.empty())
    {
        Geometry out;
        out.instances().resize(0);
        out.instances().create<Vector2i>(builtin::topo);
        return out;
    }

    vector<Vector2i> pairs;
    pairs.reserve(vert_orig.size() * 4);

    bvh.query(query_aabbs,
              [&](IndexT query_idx, IndexT compact_tri_idx)
              {
                  const IndexT v   = vert_orig[query_idx];
                  const IndexT tri = compact_to_orig_tri[compact_tri_idx];
                  Float max_d = per_vertex_max_d ? (*per_vertex_max_d)[v] : max_distance;
                  const Vector3& p  = V_pos[v];
                  const Vector3& t0 = T_pos[tri_topo[tri](0)];
                  const Vector3& t1 = T_pos[tri_topo[tri](1)];
                  const Vector3& t2 = T_pos[tri_topo[tri](2)];
                  Float d2 = point_triangle_squared_distance(p, t0, t1, t2);
                  if(d2 <= max_d * max_d)
                      pairs.emplace_back(v, tri);
              });

    Geometry result;
    result.instances().resize(pairs.size());
    auto topo_slot = result.instances().create<Vector2i>(builtin::topo);
    auto topo_view = view(*topo_slot);
    std::ranges::copy(pairs, topo_view.begin());

    return result;
}
}  // namespace uipc::geometry
