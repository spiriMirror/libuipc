#include <Eigen/Dense>
#include <uipc/geometry/utils/compute_instance_volume.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/is_trimesh_closed.h>

namespace uipc::geometry
{
static Float compute_tetmesh_volume(const SimplicialComplex& R)
{
    auto pos_view = R.positions().view();
    auto tet_view = R.tetrahedra().topo().view();

    Float volume = 0.0;

    for(auto&& [I, t] : enumerate(tet_view))
    {
        auto [p0, p1, p2, p3] =
            std::tuple{pos_view[t[0]], pos_view[t[1]], pos_view[t[2]], pos_view[t[3]]};

        Matrix<Float, 3, 3> A;
        A.col(0) = p1 - p0;
        A.col(1) = p2 - p0;
        A.col(2) = p3 - p0;
        auto D   = A.determinant();
        if(D < 0.0)
        {
            UIPC_WARN_WITH_LOCATION(
                "The determinant of the tetrahedron {} ({},{},{},{}) is non-positive ({}), "
                "which means the tetrahedron is inverted.",
                I,
                t[0],
                t[1],
                t[2],
                t[3],
                D);
        }
        volume += D / 6.0;
    }

    return volume;
}

static Float compute_trimesh_volume(const SimplicialComplex& R)
{
    auto pos_view    = R.positions().view();
    auto tri_view    = R.triangles().topo().view();
    auto orient      = R.triangles().find<IndexT>(builtin::orient);
    auto orient_view = orient ? orient->view() : span<IndexT>{};

    Float volume = 0.0;

    for(auto&& [I, t] : enumerate(tri_view))
    {
        auto [p0, p1, p2] = std::tuple{pos_view[t[0]], pos_view[t[1]], pos_view[t[2]]};

        Float orient_factor = 1.0;
        if(orient_view.size())
            orient_factor = orient_view[I];

        volume += orient_factor * p0.cross(p1).dot(p2) / 6.0;
    }

    return volume;
}


UIPC_GEOMETRY_API S<AttributeSlot<Float>> compute_instance_volume(SimplicialComplex& R)
{
    auto inst_volume = R.instances().find<Float>("volume");
    if(!inst_volume)
        inst_volume = R.instances().create<Float>("volume");

    Float volume = 0.0;

    if(R.dim() == 3)
    {
        volume = compute_tetmesh_volume(R);
    }
    else if(R.dim() == 2)
    {
        UIPC_ASSERT(is_trimesh_closed(R), "Calculating volume of open trimesh is meaningless.");

        volume = compute_trimesh_volume(R);
    }
    else
    {
        UIPC_ASSERT(false, "Only tetmesh and closed trimesh are supported.");
    }

    auto inst_volume_view = view(*inst_volume);
    std::ranges::fill(inst_volume_view, volume);

    return inst_volume;
}
}  // namespace uipc::geometry
