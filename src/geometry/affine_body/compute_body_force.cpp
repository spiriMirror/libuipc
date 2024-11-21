#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/range.h>
#include <Eigen/Dense>

namespace uipc::geometry::affine_body
{
static Vector12 apply_jacobi(const Vector3& x_bar, const Vector3& f)
{
    return Vector12{

        f.x(),  //
        f.y(),  //
        f.z(),  //

        x_bar.x() * f.x(),  //
        x_bar.y() * f.x(),  //
        x_bar.z() * f.x(),  //

        x_bar.x() * f.y(),  //
        x_bar.y() * f.y(),  //
        x_bar.z() * f.y(),  //

        x_bar.x() * f.z(),  //
        x_bar.y() * f.z(),  //
        x_bar.z() * f.z(),  //
    };
}


static Vector12 compute_tetmesh_body_force(const SimplicialComplex& sc,
                                           const Vector3& body_force_density)
{
    Vector12 body_force = Vector12::Zero();

    auto pos_view      = sc.positions().view();
    auto vertex_volume = sc.vertices().find<Float>(builtin::volume);

    UIPC_ASSERT(vertex_volume, "Volume attribute not found, why can it happen?");

    auto volume_view = vertex_volume->view();

    for(auto&& i : range(pos_view.size()))
    {
        const auto& x_bar = pos_view[i];
        auto        f     = body_force_density * volume_view[i];
        body_force += apply_jacobi(x_bar, f);
    }

    return body_force;
}

static Vector12 compute_trimesh_body_force(const SimplicialComplex& sc,
                                           const Vector3& body_force_density)
{
    Vector12 body_force = Vector12::Zero();

    auto pos_view    = sc.positions().view();
    auto tri_view    = sc.triangles().topo().view();
    auto orient      = sc.meta().find<IndexT>(builtin::orient);
    auto orient_view = orient ? orient->view() : span<const IndexT>{};

    const auto& f = body_force_density;

    // Using Divergence theorem to compute the body force
    // by integrating on the surface of the trimesh

    for(auto&& [i, F] : enumerate(tri_view))
    {
        const auto& p0 = pos_view[F[0]];
        const auto& p1 = pos_view[F[1]];
        const auto& p2 = pos_view[F[2]];

        Vector3 X = (p0 + p1 + p2) / 3.0;

        Vector3 S = (p1 - p0).cross(p2 - p0) / 2.0;
        if(orient && orient_view[i] < 0)
            S = -S;


        body_force(0) += f.x() * X.x() * S.x();  // [fx * x, 0, 0] * [Sx, Sy, Sz] = fx * x * Sx
        body_force(1) += f.y() * X.y() * S.y();  // [0, fy * y, 0] * [Sx, Sy, Sz] = fy * y * Sy
        body_force(2) += f.z() * X.z() * S.z();  // [0, 0, fz * z] * [Sx, Sy, Sz] = fz * z * Sz

        Float half_xxSx = X.x() * X.x() * S.x() / 2.0;
        Float half_yySy = X.y() * X.y() * S.y() / 2.0;
        Float half_zzSz = X.z() * X.z() * S.z() / 2.0;

        body_force(3) += f.x() * half_xxSx;
        body_force(4) += f.x() * half_yySy;
        body_force(5) += f.x() * half_zzSz;

        body_force(6) += f.y() * half_xxSx;
        body_force(7) += f.y() * half_yySy;
        body_force(8) += f.y() * half_zzSz;

        body_force(9) += f.z() * half_xxSx;
        body_force(10) += f.z() * half_yySy;
        body_force(11) += f.z() * half_zzSz;
    }

    return body_force;
}


UIPC_GEOMETRY_API Vector12 compute_body_force(const SimplicialComplex& sc,
                                              const Vector3& body_force_density)
{
    if(sc.dim() == 3)
    {
        return compute_tetmesh_body_force(sc, body_force_density);
    }
    else if(sc.dim() == 2)
    {
        return compute_trimesh_body_force(sc, body_force_density);
    }
    else
    {
        UIPC_ASSERT(false, "Only 2D and 3D SimplicialComplex is supported.");
    }
}
}  // namespace uipc::geometry::affine_body
