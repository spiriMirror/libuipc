#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/enumerate.h>
#include <Eigen/Geometry>
#include <uipc/geometry/utils/is_trimesh_closed.h>
namespace uipc::geometry
{
S<AttributeSlot<IndexT>> label_triangle_orient(SimplicialComplex& R)
{
    UIPC_ASSERT(R.dim() == 3, "label_triangle_orient() only works on 3D simplicial complex (tetmesh).");

    auto f_is_surf   = R.triangles().find<IndexT>(builtin::is_surf);
    auto f_parent_id = R.triangles().find<IndexT>(builtin::parent_id);
    auto v_position  = R.vertices().find<Vector3>(builtin::position);

    UIPC_ASSERT(f_is_surf, "Cannot find attribute `is_surf` on triangles. You may need to call `label_surface()` first");
    UIPC_ASSERT(f_parent_id, "Cannot find attribute `parent_id` on triangles. You may need to call `label_surface()` first");
    UIPC_ASSERT(v_position, "Cannot find attribute `position` on vertices. Abstract simplicial complex is not allowed!");

    auto f_orient = R.triangles().find<IndexT>(builtin::orient);
    if(!f_orient)
    {
        f_orient = R.triangles().create<IndexT>(builtin::orient, 0);
    }

    auto Fs = R.triangles().topo().view();
    auto Ts = R.tetrahedra().topo().view();

    auto parent_id_view = f_parent_id->view();
    auto is_surf_view   = f_is_surf->view();
    auto orient_view    = view(*f_orient);
    auto pos_view       = v_position->view();

    for(auto&& [i, F] : enumerate(Fs))
    {
        auto parent_id = parent_id_view[i];
        auto is_surf   = is_surf_view[i];

        const Vector4i& T = Ts[parent_id];

        // sort the vertices of the triangle and tetrahedron
        Vector3i SF = F;
        Vector4i ST = T;
        std::ranges::sort(SF);
        std::ranges::sort(ST);

        // to find the opposite vertex of the triangle
        // we just find the first mismatch element
        auto [_, It] = std::mismatch(SF.begin(), SF.end(), ST.begin());

        // so, `It` is the first element that is not in SF, which means R is the opposite vertex of the triangle
        Vector3 V = pos_view[*It];

        Vector3 A = pos_view[SF[0]];
        Vector3 B = pos_view[SF[1]];
        Vector3 C = pos_view[SF[2]];

        // check the orientation of the triangle

        Vector3 N   = (B - A).cross(C - A);
        Vector3 D   = V - A;
        Float   dot = N.dot(D);
        if(dot < 0)  // outward
        {
            orient_view[i] = 1;
        }
        else if(dot > 0)  // inward
        {
            orient_view[i] = -1;
        }
        else  // not determined
        {
            orient_view[i] = 0;
        }
    }

    return f_orient;
}
}  // namespace uipc::geometry
