#include <uipc/geometry/utils/factory.h>
#include <uipc/geometry/details/factory.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/closure.h>
#include <algorithm>

namespace uipc::geometry
{
namespace detail
{
    static void build_vertices(SimplicialComplex& sc, span<const Vector3> Vs)
    {
        auto pos_view = resize_positions(sc, Vs.size());
        std::ranges::copy(Vs, pos_view.begin());
    }
}  // namespace detail

SimplicialComplex tetmesh(span<const Vector3> Vs, span<const Vector4i> Ts)
{
    SimplicialComplex sc;

    // Create tetrahedra
    auto topo_view = detail::resize_topology(sc.tetrahedra(), Ts.size());
    std::ranges::copy(Ts, topo_view.begin());

    detail::build_vertices(sc, Vs);

    return facet_closure(sc);
}

SimplicialComplex trimesh(span<const Vector3> Vs, span<const Vector3i> Fs)
{
    SimplicialComplex sc;

    // Create triangles
    auto topo_view = detail::resize_topology(sc.triangles(), Fs.size());
    std::ranges::copy(Fs, topo_view.begin());


    detail::build_vertices(sc, Vs);

    return facet_closure(sc);
}

SimplicialComplex trimesh(span<const Vector3> Vs, span<const Vector4i> Fs)
{
    SimplicialComplex sc;
    detail::triangulate_quads(sc, Fs);
    detail::build_vertices(sc, Vs);
    return facet_closure(sc);
}

SimplicialComplex linemesh(span<const Vector3> Vs, span<const Vector2i> Es)
{
    SimplicialComplex sc;

    // Create edges
    auto topo_view = detail::resize_topology(sc.edges(), Es.size());
    std::ranges::copy(Es, topo_view.begin());

    detail::build_vertices(sc, Vs);

    return facet_closure(sc);
}

SimplicialComplex pointcloud(span<const Vector3> Vs)
{
    SimplicialComplex sc;

    detail::build_vertices(sc, Vs);

    return sc;
}

ImplicitGeometry halfplane(const Vector3& P, const Vector3& N)
{
    ImplicitGeometry ig;
    auto             uid = ig.meta().find<U64>(builtin::implicit_geometry_uid);

    // By libuipc specification: half-plane has UID 1
    constexpr auto HalfPlaneUID = 1ull;
    view(*uid)[0]               = HalfPlaneUID;

    ig.instances().create<Vector3>("N", N);
    ig.instances().create<Vector3>("P", P);
    ig.instances().create<IndexT>(builtin::is_fixed, 1);

    return ig;
}

ImplicitGeometry ground(Float height, const Vector3& N)
{
    // Create a half-plane at the given height
    // P = N * height
    return halfplane(N * height, N);
}
}  // namespace uipc::geometry
