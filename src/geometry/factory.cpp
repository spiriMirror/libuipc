#include <uipc/geometry/utils/factory.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/closure.h>
namespace uipc::geometry
{
SimplicialComplex tetmesh(span<const Vector3> Vs, span<const Vector4i> Ts)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.tetrahedra().resize(Ts.size());

    auto dst_Ts = geometry::view(asc.tetrahedra());
    std::ranges::copy(Ts, dst_Ts.begin());

    return facet_closure(SimplicialComplex{asc, Vs});
}

SimplicialComplex trimesh(span<const Vector3> Vs, span<const Vector3i> Fs)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.triangles().resize(Fs.size());

    auto dst_Ts = view(asc.triangles());
    std::ranges::copy(Fs, dst_Ts.begin());

    return facet_closure(SimplicialComplex{asc, Vs});
}

SimplicialComplex linemesh(span<const Vector3> Vs, span<const Vector2i> Es)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.edges().resize(Es.size());

    auto dst_Ts = view(asc.edges());
    std::ranges::copy(Es, dst_Ts.begin());

    return facet_closure(SimplicialComplex{asc, Vs});
}

SimplicialComplex pointcloud(span<const Vector3> Vs)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());

    return facet_closure(SimplicialComplex{asc, Vs});
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