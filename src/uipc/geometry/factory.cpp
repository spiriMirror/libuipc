#include <uipc/geometry/factory.h>

namespace uipc::geometry
{
SimplicialComplex tetmesh(span<const Vector3> Vs, span<const Vector4i> Ts)
{
    AbstractSimplicialComplex asc;
    asc.vertices()->resize(Vs.size());
    asc.tetrahedra()->resize(Ts.size());

    auto dst_Ts = asc.tetrahedra()->view();
    std::copy(Ts.begin(), Ts.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}

SimplicialComplex trimesh(span<const Vector3> Vs, span<const Vector3i> Fs)
{
    AbstractSimplicialComplex asc;
    asc.vertices()->resize(Vs.size());
    asc.triangles()->resize(Fs.size());

    auto dst_Ts = asc.triangles()->view();
    std::copy(Fs.begin(), Fs.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}

SimplicialComplex linemesh(span<const Vector3> Vs, span<const Vector2i> Es)
{
    AbstractSimplicialComplex asc;
    asc.vertices()->resize(Vs.size());
    asc.edges()->resize(Es.size());

    auto dst_Ts = asc.edges()->view();
    std::copy(Es.begin(), Es.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}

SimplicialComplex pointcloud(span<const Vector3> Vs)
{
    AbstractSimplicialComplex asc;
    asc.vertices()->resize(Vs.size());

    return SimplicialComplex{asc, Vs};
}
}  // namespace uipc::geometry
