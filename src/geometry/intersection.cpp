#include <uipc/geometry/utils/intersection.h>
#include <Eigen/Dense>
#include <igl/segment_segment_intersect.h>

namespace uipc::geometry
{
static constexpr auto eps = std::numeric_limits<Float>::epsilon();

// Returns true if segment ab intersects triangle (p0,p1,p2)
bool tri_edge_intersect(const Vector3& T0,
                        const Vector3& T1,
                        const Vector3& T2,
                        const Vector3& E0,
                        const Vector3& E1)
{
    // Check if segment and triangle are coplanar
    Vector3 n = (T1 - T0).cross(T2 - T0);
    n.normalize();
    double da = (E0 - T0).dot(n);
    double db = (E1 - T0).dot(n);

    // If both endpoints of edge are on the same side, no intersection
    if((da > 0 && db > 0) || (da < 0 && db < 0))
        return false;

    // If both are zero, edge lies in the triangle's plane (coplanar)
    if(fabs(da) < eps && fabs(db) < eps)
    {
        // Check if either endpoint is inside the triangle
        auto point_in_triangle =
            [](const Vector3& pt, const Vector3& t0, const Vector3& t1, const Vector3& t2)
        {
            Vector3 u  = t1 - t0;
            Vector3 v  = t2 - t0;
            Vector3 w  = pt - t0;
            double  uu = u.dot(u), uv = u.dot(v), vv = v.dot(v);
            double  wu = w.dot(u), wv = w.dot(v);
            double  denom = uv * uv - uu * vv;
            double  s     = (uv * wv - vv * wu) / denom;
            double  t     = (uv * wu - uu * wv) / denom;
            return (s >= 0) && (t >= 0) && (s + t <= 1);
        };
        if(point_in_triangle(E0, T0, T1, T2) || point_in_triangle(E1, T0, T1, T2))
            return true;

        Float u, v;
        // Check if edge and any triangle edge overlap (as segments)
        if(igl::segment_segment_intersect(E0, E1 - E0, T0, T1 - T0, u, v)
           || igl::segment_segment_intersect(E0, E1 - E0, T1, T2 - T1, u, v)
           || igl::segment_segment_intersect(E0, E1 - E0, T2, T0 - T2, u, v))
        {
            return true;
        }

        return false;
    }

    // Otherwise, compute intersection with the plane and check if point is inside triangle and on the edge
    double  t     = da / (da - db);
    Vector3 inter = E0 + t * (E1 - E0);

    // Check if intersection is within segment (already ensured by t in [0,1])
    if(t < 0 || t > 1.0)
        return false;

    // Check if intersection point is inside the triangle
    auto point_in_triangle =
        [](const Vector3& pt, const Vector3& t0, const Vector3& t1, const Vector3& t2)
    {
        Vector3 u  = t1 - t0;
        Vector3 v  = t2 - t0;
        Vector3 w  = pt - t0;
        double  uu = u.dot(u), uv = u.dot(v), vv = v.dot(v);
        double  wu = w.dot(u), wv = w.dot(v);
        double  denom = uv * uv - uu * vv;
        double  s     = (uv * wv - vv * wu) / denom;
        double  t     = (uv * wu - uu * wv) / denom;
        return (s >= 0) && (t >= 0) && (s + t <= 1.0);
    };

    return point_in_triangle(inter, T0, T1, T2);
}


//bool tri_edge_intersect(const Vector3& triVA,
//                        const Vector3& triVB,
//                        const Vector3& triVC,
//                        const Vector3& edgeVE,
//                        const Vector3& edgeVF,
//                        bool&          coplanar,
//                        Vector3&       uvw_in_tri,
//                        Vector2&       uv_in_edge)
//{
//    coplanar = false;
//
//    Vector3 EF = edgeVF - edgeVE;
//    Vector3 AB = triVB - triVA;
//    Vector3 AC = triVC - triVA;
//    Vector3 AE = edgeVE - triVA;
//
//    // E + EF * t = A + AB * u + AC * v
//    // => AE =  - EF * t + AB * u + AC * v
//    // solve for t, u, v
//
//    Matrix3x3 M;
//
//    M.col(0) = -EF;
//    M.col(1) = AB;
//    M.col(2) = AC;
//
//    if(std::abs(M.determinant()) <= 1e-6)  // coplanar or parallel
//    {
//        // check if E is in the plane of the triangle
//        Vector3 N       = AB.cross(AC);
//        auto    AE_on_N = N.dot(AE);
//        if(AE_on_N != 0)  // parallel
//            return false;
//
//        Vector3 BC = triVC - triVB;
//
//        coplanar = true;
//
//        Eigen::Vector2d tu;
//        bool            intersect = false;
//        intersect |=
//            igl::segment_segment_intersect(edgeVE, EF, triVA, AB, tu[0], tu[1]);
//        intersect |=
//            igl::segment_segment_intersect(edgeVE, EF, triVA, AC, tu[0], tu[1]);
//        intersect |=
//            igl::segment_segment_intersect(edgeVE, EF, triVB, BC, tu[0], tu[1]);
//
//        return intersect;
//    }
//
//    Vector3 tuv = M.inverse() * AE;
//
//    // t should be in [0, 1]
//    // u, v should be in [0, 1] and u + v <= 1
//
//    uvw_in_tri[0] = 1.0 - tuv[1] - tuv[2];
//    uvw_in_tri[1] = tuv[1];
//    uvw_in_tri[2] = tuv[2];
//
//    uv_in_edge[0] = 1.0 - tuv[0];
//    uv_in_edge[1] = tuv[0];
//
//    auto in_01 = [](double x) { return x >= 0 && x <= 1; };
//
//    bool in_tri = in_01(uvw_in_tri[0]) && in_01(uvw_in_tri[1]) && in_01(uvw_in_tri[2]);
//
//    bool in_edge = in_01(uv_in_edge[0]) && in_01(uv_in_edge[1]);
//
//    return in_tri && in_edge;
//}

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

    Matrix3x3 M;

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
