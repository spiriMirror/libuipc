#include <uipc/geometry/utils/distance.h>
#include <uipc/common/log.h>
#include <Eigen/Dense>

namespace uipc::geometry
{
namespace detail
{
    static void point_point_distance2(const Vector3& a, const Vector3& b, Float& dist2)
    {
        dist2 = (a - b).squaredNorm();
    }

    static void point_edge_distance2(const Vector3& p, const Vector3& e0, const Vector3& e1, Float& dist2)
    {
        dist2 = (e0 - p).cross(e1 - p).squaredNorm() / (e1 - e0).squaredNorm();
    }

    static void point_triangle_distance2(const Vector3& p,
                                         const Vector3& t0,
                                         const Vector3& t1,
                                         const Vector3& t2,
                                         Float&         dist2)
    {
        const Vector3 b   = (t1 - t0).cross(t2 - t0);
        Float         aTb = (p - t0).dot(b);
        dist2             = aTb * aTb / b.squaredNorm();
    }


    static void edge_edge_distance2(const Vector3& ea0,
                                    const Vector3& ea1,
                                    const Vector3& eb0,
                                    const Vector3& eb1,
                                    Float&         dist2)
    {
        Eigen::Matrix<Float, 3, 1> b =
            Eigen::Matrix<Float, 3, 1>(ea1 - ea0).cross(eb1 - eb0);
        Float aTb = Eigen::Matrix<Float, 3, 1>(eb0 - ea0).dot(b);
        dist2     = aTb * aTb / b.squaredNorm();
    }

    template <int N>
    static IndexT active_count(const Vector<IndexT, N>& flag)
    {
        IndexT count = 0;
        for(IndexT i = 0; i < N; ++i)
            count += flag[i];
        return count;
    }

    static Vector2i pp_from_pe(const Vector<IndexT, 3>& flag)
    {
        UIPC_ASSERT(active_count(flag) == 2, "active count mismatch");

        Vector<IndexT, 2> offsets;
        if(flag[0] == 0)
        {
            offsets = {1, 2};
        }
        else if(flag[1] == 0)
        {
            offsets = {0, 2};
        }
        else if(flag[2] == 0)
        {
            offsets = {0, 1};
        }
        else
        {
            UIPC_ERROR_WITH_LOCATION("Invalid flag ({},{},{})", flag[0], flag[1], flag[2]);
        }
        return offsets;
    }

    static Vector3i pe_from_pt(const Vector<IndexT, 4>& flag)
    {
        UIPC_ASSERT(active_count(flag) == 3,
                    "active count mismatch, yours=({},{},{},{})",
                    flag[0],
                    flag[1],
                    flag[2],
                    flag[3]);

        Vector<IndexT, 3> offsets;
        if(flag[0] == 0)
        {
            offsets = {1, 2, 3};
        }
        else if(flag[1] == 0)
        {
            offsets = {0, 2, 3};
        }
        else if(flag[2] == 0)
        {
            offsets = {0, 1, 3};
        }
        else if(flag[3] == 0)
        {
            offsets = {0, 1, 2};
        }
        else
        {
            UIPC_ERROR_WITH_LOCATION(
                "Invalid flag ({},{},{},{})", flag[0], flag[1], flag[2], flag[3]);
        }
        return offsets;
    }

    static Vector2i pp_from_pt(const Vector<IndexT, 4>& flag)
    {
        UIPC_ASSERT(active_count(flag) == 2,
                    "active count mismatch, yours=({},{},{},{})",
                    flag[0],
                    flag[1],
                    flag[2],
                    flag[3]);

        Vector<IndexT, 2> offsets;
        constexpr IndexT  N = 4;
        constexpr IndexT  M = 2;

        IndexT iM = 0;
        for(IndexT iN = 0; iN < N; ++iN)
        {
            if(flag[iN])
            {
                UIPC_ASSERT(iM < M, "active mismatch");
                offsets[iM] = iN;
                ++iM;
            }
        }
        return offsets;
    }

    static Vector3i pe_from_ee(const Vector<IndexT, 4>& flag)
    {
        UIPC_ASSERT(active_count(flag) == 3,
                    "active count mismatch, yours=({},{},{},{})",
                    flag[0],
                    flag[1],
                    flag[2],
                    flag[3]);

        Vector<IndexT, 3> offsets;  // [P, E0, E1]
        if(flag[0] == 0)
        {
            offsets = {1, 2, 3};
        }
        else if(flag[1] == 0)
        {
            offsets = {0, 2, 3};
        }
        else if(flag[2] == 0)
        {
            offsets = {3, 0, 1};
        }
        else if(flag[3] == 0)
        {
            offsets = {2, 0, 1};
        }
        return offsets;
    }

    static Vector2i pp_from_ee(const Vector4i& flag)
    {
        UIPC_ASSERT(active_count(flag) == 2,
                    "active count mismatch, yours=({},{},{},{})",
                    flag[0],
                    flag[1],
                    flag[2],
                    flag[3]);

        Vector<IndexT, 2> offsets;
        constexpr IndexT  N = 4;
        constexpr IndexT  M = 2;

        IndexT iM = 0;
        for(IndexT iN = 0; iN < N; ++iN)
        {
            if(flag[iN])
            {
                UIPC_ASSERT(iM < M, "active mismatch");
                offsets[iM] = iN;
                ++iM;
            }
        }
        return offsets;
    }

    static Vector<IndexT, 3> point_edge_distance_flag(const Vector3& p,
                                                      const Vector3& e0,
                                                      const Vector3& e1)
    {
        Vector<IndexT, 3> F;
        F[0] = 1;

        Vector3 e     = e1 - e0;
        auto    ratio = e.dot(p - e0) / e.squaredNorm();

        F[1] = ratio < 1.0 ? 1 : 0;
        F[2] = ratio > 0.0 ? 1 : 0;

        return F;
    }

    static Vector4i point_triangle_distance_flag(const Vector3& p,
                                                 const Vector3& t0,
                                                 const Vector3& t1,
                                                 const Vector3& t2)
    {
        Vector4i F;
        F[0] = 1;

        // clear flags
        F[1] = 0;
        F[2] = 0;
        F[3] = 0;

        //tex:
        // $$ B =
        // \begin{bmatrix}
        // T_1 - T_0 \\
        // T_2 - T_0
        // \end{bmatrix}
        // $$
        Eigen::Matrix<Float, 2, 3> basis;
        basis.row(0) = (t1 - t0).transpose();
        basis.row(1) = (t2 - t0).transpose();

        const Vector3 nVec = basis.row(0).cross(basis.row(1));

        Eigen::Matrix<Float, 2, 3> param;

        basis.row(1)                            = basis.row(0).cross(nVec);
        Eigen::Matrix<Float, 2, 2> basis_basisT = basis * basis.transpose();
        Eigen::Matrix<Float, 2, 2> invBasis     = basis_basisT.inverse();

        param.col(0) = invBasis * (basis * (p - t0));

        if(param(0, 0) > 0.0 && param(0, 0) < 1.0 && param(1, 0) >= 0.0)
        {
            // PE t0t1
            F[1] = 1;
            F[2] = 1;
        }
        else
        {
            basis.row(0) = (t2 - t1).transpose();

            basis.row(1) = basis.row(0).cross(nVec);

            Eigen::Matrix<Float, 2, 2> basis_basisT = basis * basis.transpose();
            Eigen::Matrix<Float, 2, 2> invBasis     = basis_basisT.inverse();

            param.col(1) = invBasis * (basis * (p - t1));

            if(param(0, 1) > 0.0 && param(0, 1) < 1.0 && param(1, 1) >= 0.0)
            {
                // PE t1t2
                F[2] = 1;
                F[3] = 1;
            }
            else
            {
                basis.row(0) = (t0 - t2).transpose();

                basis.row(1) = basis.row(0).cross(nVec);

                Eigen::Matrix<Float, 2, 2> basis_basisT = basis * basis.transpose();
                Eigen::Matrix<Float, 2, 2> invBasis = basis_basisT.inverse();
                param.col(2) = invBasis * (basis * (p - t2));

                if(param(0, 2) > 0.0 && param(0, 2) < 1.0 && param(1, 2) >= 0.0)
                {
                    // PE t2t0
                    F[3] = 1;
                    F[1] = 1;
                }
                else
                {
                    if(param(0, 0) <= 0.0 && param(0, 2) >= 1.0)
                    {
                        // PP t0
                        F[1] = 1;
                    }
                    else if(param(0, 1) <= 0.0 && param(0, 0) >= 1.0)
                    {
                        // PP t1
                        F[2] = 1;
                    }
                    else if(param(0, 2) <= 0.0 && param(0, 1) >= 1.0)
                    {
                        // PP t2
                        F[3] = 1;
                    }
                    else
                    {  // PT
                        F[1] = 1;
                        F[2] = 1;
                        F[3] = 1;
                    }
                }
            }
        }

        return F;
    }

    static Vector4i edge_edge_distance_flag(const Vector3& ea0,
                                            const Vector3& ea1,
                                            const Vector3& eb0,
                                            const Vector3& eb1)
    {
        Vector4i F = {1, 1, 1, 1};  // default EE

        Vector3 u  = ea1 - ea0;
        Vector3 v  = eb1 - eb0;
        Vector3 w  = ea0 - eb0;
        Float   a  = u.squaredNorm();  // always >= 0
        Float   b  = u.dot(v);
        Float   c  = v.squaredNorm();  // always >= 0
        Float   d  = u.dot(w);
        Float   e  = v.dot(w);
        Float   D  = a * c - b * b;  // always >= 0
        Float   tD = D;              // tc = tN / tD, default tD = D >= 0
        Float   sN, tN;

        // compute the line parameters of the two closest points
        sN = (b * e - c * d);
        if(sN <= 0.0)
        {  // sc < 0 => the s=0 edge is visible
            tN = e;
            tD = c;

            // PE: Ea0Eb0Eb1

            F[0] = 1;  // Ea0
            F[1] = 0;
            F[2] = 1;  // Eb0
            F[3] = 1;  // Eb1
        }
        else if(sN >= D)
        {  // sc > 1  => the s=1 edge is visible
            tN = e + b;
            tD = c;
            // PE: Ea1Eb0Eb1

            F[0] = 0;
            F[1] = 1;  // Ea1
            F[2] = 1;  // Eb0
            F[3] = 1;  // Eb1
        }
        else
        {
            tN = (a * e - b * d);
            if(tN > 0.0 && tN < tD
               && (u.cross(v).dot(w) == 0.0 || u.cross(v).squaredNorm() < 1.0e-20 * a * c))
            {
                // if (tN > 0.0 && tN < tD && (u.cross(v).dot(w) == 0.0 || u.cross(v).squaredNorm() == 0.0)) {
                // std::cout << u.cross(v).squaredNorm() / (a * c) << ": " << sN << " " << D << ", " << tN << " " << tD << std::endl;
                // avoid coplanar or nearly parallel EE
                if(sN < D / 2)
                {
                    tN = e;
                    tD = c;
                    // PE: Ea0Eb0Eb1
                    F[0] = 1;  // Ea0
                    F[1] = 0;
                    F[2] = 1;  // Eb0
                    F[3] = 1;  // Eb1
                }
                else
                {
                    tN = e + b;
                    tD = c;
                    // PE: Ea1Eb0Eb1
                    F[0] = 0;
                    F[1] = 1;  // Ea1
                    F[2] = 1;  // Eb0
                    F[3] = 1;  // Eb1
                }
            }
            // else defaultCase stays as EE
        }

        if(tN <= 0.0)
        {  // tc < 0 => the t=0 edge is visible
            // recompute sc for this edge
            if(-d <= 0.0)
            {
                // PP: Ea0Eb0
                F[0] = 1;  // Ea0
                F[1] = 0;
                F[2] = 1;  // Eb0
                F[3] = 0;
            }
            else if(-d >= a)
            {
                // PP: Ea1Eb0
                F[0] = 0;
                F[1] = 1;  // Ea1
                F[2] = 1;  // Eb0
                F[3] = 0;
            }
            else
            {
                // PE: Eb0Ea0Ea1
                F[0] = 1;  // Ea0
                F[1] = 1;  // Ea1
                F[2] = 1;  // Eb0
                F[3] = 0;
            }
        }
        else if(tN >= tD)
        {  // tc > 1  => the t=1 edge is visible
            // recompute sc for this edge
            if((-d + b) <= 0.0)
            {
                // PP: Ea0Eb1
                F[0] = 1;  // Ea0
                F[1] = 0;
                F[2] = 0;
                F[3] = 1;  // Eb1
            }
            else if((-d + b) >= a)
            {
                // PP: Ea1Eb1
                F[0] = 0;
                F[1] = 1;  // Ea1
                F[2] = 0;
                F[3] = 1;  // Eb1
            }
            else
            {
                // PE: Eb1Ea0Ea1
                F[0] = 1;  // Ea0
                F[1] = 1;  // Ea1
                F[2] = 0;
                F[3] = 1;  // Eb1
            }
        }

        return F;
    }

    static void point_edge_distance2(const Vector3i& flag,
                                     const Vector3&  p,
                                     const Vector3&  e0,
                                     const Vector3&  e1,
                                     Float&          D)
    {
        IndexT  dim = active_count(flag);
        Vector3 P[] = {p, e0, e1};

        if(dim == 2)
        {
            Vector2i offsets = pp_from_pe(flag);
            auto&    P0      = P[offsets[0]];
            auto&    P1      = P[offsets[1]];

            point_point_distance2(P0, P1, D);
        }
        else if(dim == 3)
        {
            point_edge_distance2(p, e0, e1, D);
        }
        else
        {
            UIPC_ERROR_WITH_LOCATION("Invalid flag ({},{},{})", flag[0], flag[1], flag[2]);
        }
    }

    static void point_triangle_distance2(const Vector4i& flag,
                                         const Vector3&  p,
                                         const Vector3&  t0,
                                         const Vector3&  t1,
                                         const Vector3&  t2,
                                         Float&          D)
    {
        IndexT  dim = active_count(flag);
        Vector3 P[] = {p, t0, t1, t2};

        if(dim == 2)
        {
            Vector2i offsets = pp_from_pt(flag);
            auto&    P0      = P[offsets[0]];
            auto&    P1      = P[offsets[1]];

            point_point_distance2(P0, P1, D);
        }
        else if(dim == 3)
        {
            Vector3i offsets = pe_from_pt(flag);
            auto&    P0      = P[offsets[0]];
            auto&    P1      = P[offsets[1]];
            auto&    P2      = P[offsets[2]];

            point_edge_distance2(P0, P1, P2, D);
        }
        else if(dim == 4)
        {
            point_triangle_distance2(p, t0, t1, t2, D);
        }
        else
        {
            UIPC_ERROR_WITH_LOCATION(
                "Invalid flag ({},{},{},{})", flag[0], flag[1], flag[2], flag[3]);
        }
    }

    static void edge_edge_distance2(const Vector4i& flag,
                                    const Vector3&  ea0,
                                    const Vector3&  ea1,
                                    const Vector3&  eb0,
                                    const Vector3&  eb1,
                                    Float&          D)
    {
        IndexT  dim = detail::active_count(flag);
        Vector3 P[] = {ea0, ea1, eb0, eb1};

        if(dim == 2)
        {
            Vector2i offsets = detail::pp_from_ee(flag);
            auto&    P0      = P[offsets[0]];
            auto&    P1      = P[offsets[1]];

            point_point_distance2(P0, P1, D);
        }
        else if(dim == 3)
        {
            Vector3i offsets = detail::pe_from_ee(flag);
            auto&    P0      = P[offsets[0]];
            auto&    P1      = P[offsets[1]];
            auto&    P2      = P[offsets[2]];

            point_edge_distance2(P0, P1, P2, D);
        }
        else if(dim == 4)
        {
            edge_edge_distance2(ea0, ea1, eb0, eb1, D);
        }
        else
        {
            UIPC_ERROR_WITH_LOCATION(
                "Invalid flag ({},{},{},{})", flag[0], flag[1], flag[2], flag[3]);
        }
    }
}  // namespace detail


Float halfplane_vertex_signed_distance(const Vector3& P, const Vector3& N, const Vector3& V, Float V_thickness)
{
    return (V - P).dot(N) - V_thickness;
}

Float point_point_squared_distance(const Vector3& a, const Vector3& b)
{
    Float dist2;
    detail::point_point_distance2(a, b, dist2);
    return dist2;
}

Float point_edge_squared_distance(const Vector3& P, const Vector3& E0, const Vector3& E1)
{
    Float dist2;

    auto F = detail::point_edge_distance_flag(P, E0, E1);
    detail::point_edge_distance2(F, P, E0, E1, dist2);

    return dist2;
}

Float point_triangle_squared_distance(const Vector3& P,
                                      const Vector3& T0,
                                      const Vector3& T1,
                                      const Vector3& T2)
{
    Float dist2;

    auto F = detail::point_triangle_distance_flag(P, T0, T1, T2);
    detail::point_triangle_distance2(F, P, T0, T1, T2, dist2);

    return dist2;
}

Float edge_edge_squared_distance(const Vector3& Ea0,
                                 const Vector3& Ea1,
                                 const Vector3& Eb0,
                                 const Vector3& Eb1)
{
    Float dist2;

    auto F = detail::edge_edge_distance_flag(Ea0, Ea1, Eb0, Eb1);
    detail::edge_edge_distance2(F, Ea0, Ea1, Eb0, Eb1, dist2);

    return dist2;
}
}  // namespace uipc::geometry
