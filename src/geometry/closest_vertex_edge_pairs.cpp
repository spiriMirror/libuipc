#include <uipc/geometry/utils/closest_vertex_edge_pairs.h>
#include <uipc/geometry/utils/distance.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/attribute_friend.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/vector.h>
#include <ranges>

namespace uipc::geometry
{
namespace
{
    using AABB = BVH::AABB;

    AABB edge_aabb(span<const Vector3> positions, const Vector2i& edge)
    {
        AABB box;
        box.extend(positions[edge(0)]);
        box.extend(positions[edge(1)]);
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

Geometry closest_vertex_edge_pairs(const SimplicialComplex& vertex_mesh,
                                   const SimplicialComplex& edge_mesh,
                                   Float                    max_distance)
{
    const auto  V_pos     = vertex_mesh.positions().view();
    const auto  edge_topo = edge_mesh.edges().topo().view();
    const auto  E_pos     = edge_mesh.positions().view();
    const SizeT nV        = V_pos.size();
    const SizeT nEdge     = edge_topo.size();

    vector<AABB> edge_aabbs;
    edge_aabbs.reserve(nEdge);
    for(IndexT i : std::views::iota(0, static_cast<IndexT>(nEdge)))
        edge_aabbs.push_back(edge_aabb(E_pos, edge_topo[i]));

    if(edge_aabbs.empty())
    {
        Geometry out;
        out.instances().resize(0);
        out.instances().create<Vector2i>(builtin::topo);
        return out;
    }

    BVH bvh;
    bvh.build(edge_aabbs);

    vector<IndexT> vert_orig;
    vector<AABB>   query_aabbs;
    vert_orig.reserve(nV);
    query_aabbs.reserve(nV);
    for(IndexT v : std::views::iota(0, static_cast<IndexT>(nV)))
    {
        vert_orig.push_back(v);
        query_aabbs.push_back(point_aabb(V_pos[v], max_distance));
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
              [&](IndexT query_idx, IndexT edge_idx)
              {
                  const IndexT   v  = vert_orig[query_idx];
                  const Vector3& p  = V_pos[v];
                  const Vector3& e0 = E_pos[edge_topo[edge_idx](0)];
                  const Vector3& e1 = E_pos[edge_topo[edge_idx](1)];
                  Float          d2 = point_edge_squared_distance(p, e0, e1);
                  if(d2 <= max_distance * max_distance)
                      pairs.emplace_back(v, edge_idx);
              });

    Geometry result;
    result.instances().resize(pairs.size());
    auto topo_slot = result.instances().create<Vector2i>(builtin::topo);
    auto topo_view = view(*topo_slot);
    std::ranges::copy(pairs, topo_view.begin());

    return result;
}
}  // namespace uipc::geometry
