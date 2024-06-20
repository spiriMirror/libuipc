#include <uipc/geometry/utils/compute_vertex_mass.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>
#include <Eigen/Dense>

namespace uipc::geometry
{
static P<AttributeSlot<Float>> compute_vertex_mass_from_tet(SimplicialComplex& R, Float mass_density)
{
    vector<Float> tet_mass;
    tet_mass.resize(R.tetrahedra().size(), 0.0);

    auto Vs = R.positions().view();
    auto Ts = R.tetrahedra().topo().view();

    std::ranges::transform(Ts,
                           tet_mass.begin(),
                           [&](const Vector4i& t) -> Float
                           {
                               auto [p0, p1, p2, p3] =
                                   std::tuple{Vs[t[0]], Vs[t[1]], Vs[t[2]], Vs[t[3]]};

                               Matrix<Float, 3, 3> A;
                               A.col(0) = p1 - p0;
                               A.col(1) = p2 - p0;
                               A.col(2) = p3 - p0;
                               auto D   = A.determinant();
                               UIPC_ASSERT(D > 0.0,
                                           "The determinant of the tetrahedron is non-positive ({}), which means the tetrahedron is inverted.",
                                           D);
                               auto volume = D / 6.0;
                               return volume * mass_density;
                           });

    auto Vm = R.vertices().find<Float>(builtin::mass);

    if(!Vm)
        Vm = R.vertices().create<Float>(builtin::mass);

    auto Vm_view = view(*Vm);

    std::ranges::fill(Vm_view, 0.0);

    for(auto&& [i, t] : enumerate(Ts))
    {
        for(auto&& v : t)
        {
            Vm_view[v] += tet_mass[i] / 4.0;
        }
    }

    return Vm;
}

static P<AttributeSlot<Float>> compute_vertex_mass_from_tri(SimplicialComplex& R, Float mass_density)
{
    vector<Float> tri_mass;
    tri_mass.resize(R.triangles().size(), 0.0);

    auto Vs = R.positions().view();
    auto Ts = R.triangles().topo().view();

    std::ranges::transform(
        Ts,
        tri_mass.begin(),
        [&](const Vector3i& t) -> Float
        {
            auto [p0, p1, p2] = std::tuple{Vs[t[0]], Vs[t[1]], Vs[t[2]]};

            auto n    = (p1 - p0).cross(p2 - p0);
            auto area = 0.5 * n.norm();
            return area * mass_density;
        });

    auto Vm = R.vertices().find<Float>(builtin::mass);

    if(!Vm)
        Vm = R.vertices().create<Float>(builtin::mass);

    auto Vm_view = view(*Vm);

    std::ranges::fill(Vm_view, 0.0);

    for(auto&& [i, t] : enumerate(Ts))
    {
        for(auto&& v : t)
        {
            Vm_view[v] += tri_mass[i] / 3.0;
        }
    }

    return Vm;
}

static P<AttributeSlot<Float>> compute_vertex_mass_from_edge(SimplicialComplex& R, Float mass_density)
{
    vector<Float> edge_mass;
    edge_mass.resize(R.edges().size(), 0.0);

    auto Vs = R.positions().view();
    auto Es = R.edges().topo().view();

    std::ranges::transform(Es,
                           edge_mass.begin(),
                           [&](const Vector2i& e) -> Float
                           {
                               auto [p0, p1] = std::tuple{Vs[e[0]], Vs[e[1]]};

                               auto l = (p1 - p0).norm();
                               return l * mass_density;
                           });

    auto Vm = R.vertices().find<Float>(builtin::mass);

    if(!Vm)
        Vm = R.vertices().create<Float>(builtin::mass);

    auto Vm_view = view(*Vm);

    std::ranges::fill(Vm_view, 0.0);

    for(auto&& [i, e] : enumerate(Es))
    {
        for(auto&& v : e)
        {
            Vm_view[v] += edge_mass[i] / 2.0;
        }
    }

    return Vm;
}

static P<AttributeSlot<Float>> compute_vertex_mass_from_vertex(SimplicialComplex& R, Float mass_density)
{
    auto Vs = R.positions().view();
    auto Vm = R.vertices().find<Float>(builtin::mass);

    if(!Vm)
        Vm = R.vertices().create<Float>(builtin::mass);

    auto Vm_view = view(*Vm);

    std::ranges::fill(Vm_view, mass_density);

    return Vm;
}

P<AttributeSlot<Float>> compute_vertex_mass(SimplicialComplex& R, Float mass_density)
{
    switch(R.dim())
    {
        case 0:
            return compute_vertex_mass_from_vertex(R, mass_density);
        case 1:
            return compute_vertex_mass_from_edge(R, mass_density);
        case 2:
            return compute_vertex_mass_from_tri(R, mass_density);
        case 3:
            return compute_vertex_mass_from_tet(R, mass_density);
        default: {
            UIPC_ASSERT(false, "Unsupported dimension: {}", R.dim());
            return {};
        }
    }
}
}  // namespace uipc::geometry
