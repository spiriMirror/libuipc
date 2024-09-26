#include <uipc/geometry/utils/factory.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/closure.h>
namespace uipc::geometry
{
namespace detail
{
    static void create_vertices(SimplicialComplex& sc, span<const Vector3> Vs)
    {
        sc.vertices().resize(Vs.size());
        auto pos =
            sc.vertices().create<Vector3, false>(builtin::position, Vector3::Zero());
        auto pos_view = view(*pos);
        std::ranges::copy(Vs, pos_view.begin());
    }
}  // namespace detail

SimplicialComplex tetmesh(span<const Vector3> Vs, span<const Vector4i> Ts)
{
    SimplicialComplex sc;

    // Create tetrahedra
    sc.tetrahedra().resize(Ts.size());
    auto topo = sc.tetrahedra().create<Vector4i, false>(builtin::topo, Vector4i::Zero());
    auto topo_view = view(*topo);
    std::ranges::copy(Ts, topo_view.begin());

    detail::create_vertices(sc, Vs);

    return facet_closure(sc);
}

SimplicialComplex trimesh(span<const Vector3> Vs, span<const Vector3i> Fs)
{
    SimplicialComplex sc;

    // Create triangles
    sc.triangles().resize(Fs.size());
    auto topo = sc.triangles().create<Vector3i, false>(builtin::topo, Vector3i::Zero());
    auto topo_view = view(*topo);
    std::ranges::copy(Fs, topo_view.begin());


    detail::create_vertices(sc, Vs);

    return facet_closure(sc);
}

SimplicialComplex linemesh(span<const Vector3> Vs, span<const Vector2i> Es)
{
    SimplicialComplex sc;

    // Create edges
    sc.edges().resize(Es.size());
    auto topo = sc.edges().create<Vector2i, false>(builtin::topo, Vector2i::Zero());
    auto topo_view = view(*topo);
    std::ranges::copy(Es, topo_view.begin());

    detail::create_vertices(sc, Vs);

    return facet_closure(sc);
}

SimplicialComplex pointcloud(span<const Vector3> Vs)
{
    SimplicialComplex sc;

    detail::create_vertices(sc, Vs);

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
    ImplicitGeometry hp = halfplane(Vector3::UnitY() * height, N);
    return hp;
}
}  // namespace uipc::geometry
