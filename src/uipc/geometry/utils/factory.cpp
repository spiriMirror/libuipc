#include <uipc/geometry/factory.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
SimplicialComplex tetmesh(span<const Vector3> Vs, span<const Vector4i> Ts)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.tetrahedra().resize(Ts.size());

    auto dst_Ts = geometry::view(asc.tetrahedra());
    std::copy(Ts.begin(), Ts.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}

SimplicialComplex trimesh(span<const Vector3> Vs, span<const Vector3i> Fs)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.triangles().resize(Fs.size());

    auto dst_Ts = view(asc.triangles());
    std::copy(Fs.begin(), Fs.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}

SimplicialComplex linemesh(span<const Vector3> Vs, span<const Vector2i> Es)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.edges().resize(Es.size());

    auto dst_Ts = view(asc.edges());
    std::copy(Es.begin(), Es.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}

SimplicialComplex pointcloud(span<const Vector3> Vs)
{
    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());

    return SimplicialComplex{asc, Vs};
}

ImplicitGeometry ground(Float height)
{
    ImplicitGeometry ig;
    auto             uid = ig.meta().find<U64>(builtin::implicit_geometry_uid);

    constexpr auto HalfPlaneUID = 1ull;  // By libuipc specification
    view(*uid)[0]               = HalfPlaneUID;

    ig.instances().create<Vector3>("N", Vector3{0.0, 1.0, 0.0});
    ig.instances().create<Vector3>("P", Vector3{0.0, height, 0.0});

    return ig;
}
}  // namespace uipc::geometry
