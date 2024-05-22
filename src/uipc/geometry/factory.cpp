#include <uipc/geometry/factory.h>

namespace uipc::geometry
{
SimplicialComplex tetmesh(std::span<const Vector3> Vs, std::span<const Vector4i> Ts)
{
    AbstractSimplicialComplex asc;
    asc.vertices()->resize(Vs.size());
    asc.tetrahedra()->resize(Ts.size());

    auto dst_Ts = asc.tetrahedra()->view();
    std::copy(Ts.begin(), Ts.end(), dst_Ts.begin());

    return SimplicialComplex{asc, Vs};
}
}  // namespace uipc::geometry
