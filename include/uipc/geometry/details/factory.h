#pragma once
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry::detail
{
inline span<Vector3> resize_positions(SimplicialComplex& sc, SizeT count)
{
    sc.vertices().resize(count);
    return view(sc.positions());
}

template <IndexT N>
auto resize_topology(SimplicialComplexAttributes<false, N> facets, SizeT count)
{
    using Topology = typename decltype(facets)::TopoValueT;
    facets.resize(count);
    return view(*facets.template create<Topology>(builtin::topo, Topology::Zero(), false));
}

template <typename Quads>
void triangulate_quads(SimplicialComplex& sc, const Quads& quads)
{
    auto triangles = resize_topology(sc.triangles(), quads.size() * 2);
    for(SizeT i = 0; i < quads.size(); ++i)
    {
        const auto& f        = quads[i];
        triangles[2 * i]     = Vector3i{f[0], f[1], f[2]};
        triangles[2 * i + 1] = Vector3i{f[0], f[2], f[3]};
    }
}
}  // namespace uipc::geometry::detail
