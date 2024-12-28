#include <uipc/geometry/utils/intersection.h>
#include <Eigen/Dense>
#include <igl/segment_segment_intersect.h>

namespace uipc::geometry
{
bool tri_edge_intersect(const Vector3& triVA,
                        const Vector3& triVB,
                        const Vector3& triVC,
                        const Vector3& edgeVE,
                        const Vector3& edgeVF,
                        bool&          coplanar,
                        Vector3&       uvw_in_tri,
                        Vector2&       uv_in_edge)
{
    coplanar = false;

    Eigen ::Vector3d EF = edgeVF - edgeVE;
    Vector3          AB = triVB - triVA;
    Vector3          AC = triVC - triVA;
    Vector3          AE = edgeVE - triVA;

    // E + EF * t = A + AB * u + AC * v
    // => AE =  - EF * t + AB * u + AC * v
    // solve for t, u, v

    Eigen::Matrix3d M;

    M.col(0) = -EF;
    M.col(1) = AB;
    M.col(2) = AC;

    if(std::abs(M.determinant()) <= 1e-6)  // coplanar or parallel
    {
        // check if E is in the plane of the triangle
        Eigen::Vector3d N       = AB.cross(AC);
        auto            AE_on_N = N.dot(AE);
        if(AE_on_N != 0)  // parallel
            return false;

        Vector3 BC = triVC - triVB;

        coplanar = true;

        Vector2 tu;
        bool    intersect = false;
        intersect |=
            igl::segment_segment_intersect(edgeVE, EF, triVA, AB, tu[0], tu[1]);
        intersect |=
            igl::segment_segment_intersect(edgeVE, EF, triVA, AC, tu[0], tu[1]);
        intersect |=
            igl::segment_segment_intersect(edgeVE, EF, triVB, BC, tu[0], tu[1]);

        return intersect;
    }

    Vector3 tuv = M.inverse() * AE;

    // t should be in [0, 1]
    // u, v should be in [0, 1] and u + v <= 1

    uvw_in_tri[0] = 1.0 - tuv[1] - tuv[2];
    uvw_in_tri[1] = tuv[1];
    uvw_in_tri[2] = tuv[2];

    uv_in_edge[0] = 1.0 - tuv[0];
    uv_in_edge[1] = tuv[0];

    auto in_01 = [](double x) { return x >= 0 && x <= 1; };

    bool in_tri = in_01(uvw_in_tri[0]) && in_01(uvw_in_tri[1]) && in_01(uvw_in_tri[2]);

    bool in_edge = in_01(uv_in_edge[0]) && in_01(uv_in_edge[1]);

    return in_tri && in_edge;
}

bool is_point_in_tet(const Vector3& tetVA,
                     const Vector3& tetVB,
                     const Vector3& tetVC,
                     const Vector3& tetVD,
                     const Vector3& point,
                     Vector4&       tuvw_in_tet)
{
    // P = A * (1-u-v-w) + B * u + C * v + D * w
    // AP = AB * u + AC * v + AD * w
    // solve for u, v, w

    Vector3 AB = tetVB - tetVA;
    Vector3 AC = tetVC - tetVA;
    Vector3 AD = tetVD - tetVA;
    Vector3 AP = point - tetVA;

    Eigen::Matrix3d M;

    M.col(0) = AB;
    M.col(1) = AC;
    M.col(2) = AD;

    if(M.determinant() == 0)
    {
        return false;
    }

    Vector3 uvw = M.inverse() * AP;

    auto in_01 = [](double x) { return x >= 0 && x <= 1; };

    tuvw_in_tet[0] = 1.0 - uvw[0] - uvw[1] - uvw[2];
    tuvw_in_tet[1] = uvw[0];
    tuvw_in_tet[2] = uvw[1];
    tuvw_in_tet[3] = uvw[2];

    return in_01(tuvw_in_tet[0]) && in_01(tuvw_in_tet[1])
           && in_01(tuvw_in_tet[2]) && in_01(tuvw_in_tet[3]);
}
}  // namespace uipc::geometry
