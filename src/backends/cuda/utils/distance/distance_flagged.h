#pragma once
#include <type_define.h>
#include <utils/distance/point_point.h>
#include <utils/distance/point_edge.h>
#include <utils/distance/point_triangle.h>
#include <utils/distance/edge_edge.h>
#include <muda/ext/eigen/inverse.h>
#include <muda/tools/debug_log.h>

namespace uipc::backend::cuda::distance
{
namespace detail
{
    template <int N>
    MUDA_GENERIC IndexT active_count(const Vector<IndexT, N>& flag)
    {
        IndexT count = 0;
#pragma unroll
        for(IndexT i = 0; i < N; ++i)
            count += flag[i];
        return count;
    }

    inline MUDA_GENERIC Vector<IndexT, 2> pp_from_pe(const Vector<IndexT, 3>& flag)
    {
        MUDA_ASSERT(detail::active_count(flag) == 2, "active count mismatch");

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
            MUDA_ERROR_WITH_LOCATION("Invalid flag (%d,%d,%d)", flag[0], flag[1], flag[2]);
        }
        return offsets;
    }

    inline MUDA_GENERIC Vector<IndexT, 3> pe_from_pt(const Vector<IndexT, 4>& flag)
    {
        MUDA_ASSERT(detail::active_count(flag) == 3,
                    "active count mismatch, yours=(%d,%d,%d,%d)",
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
            MUDA_ERROR_WITH_LOCATION(
                "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
        }
        return offsets;
    }

    inline MUDA_GENERIC Vector<IndexT, 2> pp_from_pt(const Vector<IndexT, 4>& flag)
    {
        MUDA_ASSERT(detail::active_count(flag) == 2,
                    "active count mismatch, yours=(%d,%d,%d,%d)",
                    flag[0],
                    flag[1],
                    flag[2],
                    flag[3]);

        Vector<IndexT, 2> offsets;
        constexpr IndexT  N = 4;
        constexpr IndexT  M = 2;

        IndexT iM = 0;
#pragma unroll
        for(IndexT iN = 0; iN < N; ++iN)
        {
            if(flag[iN])
            {
                MUDA_ASSERT(iM < M, "active mismatch");
                offsets[iM] = iN;
                ++iM;
            }
        }
        return offsets;
    }

    inline MUDA_GENERIC Vector<IndexT, 3> pe_from_ee(const Vector<IndexT, 4>& flag)
    {
        MUDA_ASSERT(detail::active_count(flag) == 3,
                    "active count mismatch, yours=(%d,%d,%d,%d)",
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

    inline MUDA_GENERIC Vector<IndexT, 2> pp_from_ee(const Vector<IndexT, 4>& flag)
    {
        MUDA_ASSERT(detail::active_count(flag) == 2,
                    "active count mismatch, yours=(%d,%d,%d,%d)",
                    flag[0],
                    flag[1],
                    flag[2],
                    flag[3]);

        Vector<IndexT, 2> offsets;
        constexpr IndexT  N = 4;
        constexpr IndexT  M = 2;

        IndexT iM = 0;
#pragma unroll
        for(IndexT iN = 0; iN < N; ++iN)
        {
            if(flag[iN])
            {
                MUDA_ASSERT(iM < M, "active mismatch");
                offsets[iM] = iN;
                ++iM;
            }
        }
        return offsets;
    }
}  // namespace detail


MUDA_GENERIC inline IndexT degenerate_point_triangle(const Vector<IndexT, 4>& flag,
                                                     Vector<IndexT, 4>& offsets)
{
    // collect active indices
    IndexT dim = detail::active_count(flag);
    if(dim == 2)
    {
        offsets.head<2>() = detail::pp_from_pt(flag);
    }
    else if(dim == 3)
    {
        offsets.head<3>() = detail::pe_from_pt(flag);
    }
    else if(dim == 4)
    {
        offsets = {0, 1, 2, 3};
    }
    return dim;
}

MUDA_GENERIC inline IndexT degenerate_edge_edge(const Vector<IndexT, 4>& flag,
                                                Vector<IndexT, 4>& offsets)
{
    // collect active indices
    IndexT dim = detail::active_count(flag);
    if(dim == 2)
    {
        offsets.head<2>() = detail::pp_from_ee(flag);
    }
    else if(dim == 3)
    {
        offsets.head<3>() = detail::pe_from_ee(flag);
    }
    else if(dim == 4)
    {
        offsets = {0, 1, 2, 3};
    }
    return dim;
}

MUDA_GENERIC inline IndexT degenerate_point_edge(const Vector<IndexT, 3>& flag,
                                                 Vector<IndexT, 3>& offsets)
{
    // collect active indices
    IndexT dim = detail::active_count(flag);
    if(dim == 2)
    {
        offsets.head<2>() = detail::pp_from_pe(flag);
    }
    else if(dim == 3)
    {
        offsets = {0, 1, 2};
    }
    return dim;
}

template <typename T>
MUDA_GENERIC Vector<IndexT, 2> point_point_distance_flag(const Eigen::Vector<T, 3>& p0,
                                                         const Eigen::Vector<T, 3>& p1)
{
    return Vector<IndexT, 2>{1, 1};
}

template <typename T>
MUDA_GENERIC Vector<IndexT, 3> point_edge_distance_flag(const Eigen::Vector<T, 3>& p,
                                                        const Eigen::Vector<T, 3>& e0,
                                                        const Eigen::Vector<T, 3>& e1)
{
    Vector<IndexT, 3> F;
    F[0] = 1;

    Vector<T, 3> e     = e1 - e0;
    auto         ratio = e.dot(p - e0) / e.squaredNorm();

    F[1] = ratio < 1.0 ? 1 : 0;
    F[2] = ratio > 0.0 ? 1 : 0;

    return F;
}

template <typename T>
MUDA_GENERIC Vector4i point_triangle_distance_flag(const Eigen::Vector<T, 3>& p,
                                                   const Eigen::Vector<T, 3>& t0,
                                                   const Eigen::Vector<T, 3>& t1,
                                                   const Eigen::Vector<T, 3>& t2)
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
    Eigen::Matrix<T, 2, 3> basis;
    basis.row(0) = (t1 - t0).transpose();
    basis.row(1) = (t2 - t0).transpose();

    const Eigen::Vector<T, 3> nVec = basis.row(0).cross(basis.row(1));

    Eigen::Matrix<T, 2, 3> param;

    basis.row(1)                        = basis.row(0).cross(nVec);
    Eigen::Matrix<T, 2, 2> basis_basisT = basis * basis.transpose();
    auto                   invBasis     = muda::eigen::inverse(basis_basisT);

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

        Eigen::Matrix<T, 2, 2> basis_basisT = basis * basis.transpose();
        auto                   invBasis = muda::eigen::inverse(basis_basisT);

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

            Eigen::Matrix<T, 2, 2> basis_basisT = basis * basis.transpose();
            auto invBasis = muda::eigen::inverse(basis_basisT);
            param.col(2)  = invBasis * (basis * (p - t2));

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

template <typename T>
MUDA_GENERIC Vector4i edge_edge_distance_flag(const Eigen::Vector<T, 3>& ea0,
                                              const Eigen::Vector<T, 3>& ea1,
                                              const Eigen::Vector<T, 3>& eb0,
                                              const Eigen::Vector<T, 3>& eb1)
{
    Vector4i F = {1, 1, 1, 1};  // default EE

    Eigen::Vector<T, 3> u  = ea1 - ea0;
    Eigen::Vector<T, 3> v  = eb1 - eb0;
    Eigen::Vector<T, 3> w  = ea0 - eb0;
    T                   a  = u.squaredNorm();  // always >= 0
    T                   b  = u.dot(v);
    T                   c  = v.squaredNorm();  // always >= 0
    T                   d  = u.dot(w);
    T                   e  = v.dot(w);
    T                   D  = a * c - b * b;  // always >= 0
    T                   tD = D;  // tc = tN / tD, default tD = D >= 0
    T                   sN, tN;

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

template <typename T>
MUDA_GENERIC void point_point_distance2(const Vector2i&            flag,
                                        const Eigen::Vector<T, 3>& a,
                                        const Eigen::Vector<T, 3>& b,
                                        T&                         D)
{
    point_point_distance2(a, b, D);
}

template <typename T>
MUDA_GENERIC void point_edge_distance2(const Vector<IndexT, 3>&   flag,
                                       const Eigen::Vector<T, 3>& p,
                                       const Eigen::Vector<T, 3>& e0,
                                       const Eigen::Vector<T, 3>& e1,
                                       T&                         D)
{
    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {p, e0, e1};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_pe(flag);
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
        MUDA_ERROR_WITH_LOCATION("Invalid flag (%d,%d,%d)", flag[0], flag[1], flag[2]);
    }
}

template <typename T>
MUDA_GENERIC void point_triangle_distance2(const Vector4i&            flag,
                                           const Eigen::Vector<T, 3>& p,
                                           const Eigen::Vector<T, 3>& t0,
                                           const Eigen::Vector<T, 3>& t1,
                                           const Eigen::Vector<T, 3>& t2,
                                           T&                         D)
{
    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {p, t0, t1, t2};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_pt(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];

        point_point_distance2(P0, P1, D);
    }
    else if(dim == 3)
    {
        Vector3i offsets = detail::pe_from_pt(flag);
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
        MUDA_ERROR_WITH_LOCATION(
            "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
    }
}

template <typename T>
MUDA_GENERIC void edge_edge_distance2(const Vector4i&            flag,
                                      const Eigen::Vector<T, 3>& ea0,
                                      const Eigen::Vector<T, 3>& ea1,
                                      const Eigen::Vector<T, 3>& eb0,
                                      const Eigen::Vector<T, 3>& eb1,
                                      T&                         D)
{
    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {ea0, ea1, eb0, eb1};

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
        MUDA_ERROR_WITH_LOCATION(
            "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
    }
}

template <typename T>
MUDA_GENERIC void point_point_distance2_gradient(const Vector2i& flag,
                                                 const Eigen::Vector<T, 3>& a,
                                                 const Eigen::Vector<T, 3>& b,
                                                 Eigen::Vector<T, 6>&       G)
{
    G.template segment<3>(0) = 2.0 * (a - b);
    G.template segment<3>(3) = -G.template segment<3>(0);
}

template <typename T>
MUDA_GENERIC void point_edge_distance2_gradient(const Vector<IndexT, 3>&   flag,
                                                const Eigen::Vector<T, 3>& p,
                                                const Eigen::Vector<T, 3>& e0,
                                                const Eigen::Vector<T, 3>& e1,
                                                Eigen::Vector<T, 9>&       G)
{
    G.setZero();

    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {p, e0, e1};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_pe(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];

        Vector<T, 6> G6;
        point_point_distance2_gradient(P0, P1, G6);

#pragma unroll
        for(int i = 0; i < 2; ++i)
            G.segment<3>(offsets[i] * 3) = G6.segment<3>(i * 3);
    }
    else if(dim == 3)
    {
        details::g_PE3D(p[0], p[1], p[2], e0[0], e0[1], e0[2], e1[0], e1[1], e1[2], G.data());
    }
    else
    {
        MUDA_ERROR_WITH_LOCATION("Invalid flag (%d,%d,%d)", flag[0], flag[1], flag[2]);
    }
}

template <typename T>
MUDA_GENERIC void point_triangle_distance2_gradient(const Vector4i& flag,
                                                    const Eigen::Vector<T, 3>& p,
                                                    const Eigen::Vector<T, 3>& t0,
                                                    const Eigen::Vector<T, 3>& t1,
                                                    const Eigen::Vector<T, 3>& t2,
                                                    Eigen::Vector<T, 12>& G)
{
    G.setZero();

    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {p, t0, t1, t2};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_pt(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];

        Vector<T, 6> G6;
        point_point_distance2_gradient(P0, P1, G6);

#pragma unroll
        for(int i = 0; i < 2; ++i)
            G.segment<3>(offsets[i] * 3) = G6.segment<3>(i * 3);
    }
    else if(dim == 3)
    {
        Vector3i offsets = detail::pe_from_pt(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        auto&    P2      = P[offsets[2]];

        Vector<T, 9> G9;
        point_edge_distance2_gradient(P0, P1, P2, G9);

#pragma unroll
        for(int i = 0; i < 3; ++i)
            G.segment<3>(offsets[i] * 3) = G9.segment<3>(i * 3);
    }
    else if(dim == 4)
    {
        details::g_PT(
            p[0], p[1], p[2], t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], G.data());
    }
    else
    {
        MUDA_ERROR_WITH_LOCATION(
            "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
    }
}

template <typename T>
MUDA_GENERIC void edge_edge_distance2_gradient(const Vector4i&            flag,
                                               const Eigen::Vector<T, 3>& ea0,
                                               const Eigen::Vector<T, 3>& ea1,
                                               const Eigen::Vector<T, 3>& eb0,
                                               const Eigen::Vector<T, 3>& eb1,
                                               Eigen::Vector<T, 12>&      G)
{
    G.setZero();

    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {ea0, ea1, eb0, eb1};

    if(dim == 2)
    {
        Vector2i     offsets = detail::pp_from_ee(flag);
        auto&        P0      = P[offsets[0]];
        auto&        P1      = P[offsets[1]];
        Vector<T, 6> G6;
        point_point_distance2_gradient(P0, P1, G6);

#pragma unroll
        for(int i = 0; i < 2; ++i)
            G.segment<3>(offsets[i] * 3) = G6.segment<3>(i * 3);
    }
    else if(dim == 3)
    {
        Vector3i offsets = detail::pe_from_ee(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        auto&    P2      = P[offsets[2]];

        Vector<T, 9> G9;
        point_edge_distance2_gradient(P0, P1, P2, G9);

#pragma unroll
        for(int i = 0; i < 3; ++i)
            G.segment<3>(offsets[i] * 3) = G9.segment<3>(i * 3);
    }
    else if(dim == 4)
    {
        details::g_EE(ea0[0],
                      ea0[1],
                      ea0[2],
                      ea1[0],
                      ea1[1],
                      ea1[2],
                      eb0[0],
                      eb0[1],
                      eb0[2],
                      eb1[0],
                      eb1[1],
                      eb1[2],
                      G.data());
    }
    else
    {
        MUDA_ERROR_WITH_LOCATION(
            "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
    }
}

template <typename T>
MUDA_GENERIC void point_point_distance2_hessian(const Vector2i&            flag,
                                                const Eigen::Vector<T, 3>& a,
                                                const Eigen::Vector<T, 3>& b,
                                                Eigen::Matrix<T, 6, 6>&    H)
{
    H.setZero();
    H.diagonal().setConstant(2.0);
    H(0, 3) = H(1, 4) = H(2, 5) = H(3, 0) = H(4, 1) = H(5, 2) = -2.0;
}

template <typename T>
MUDA_GENERIC void point_edge_distance2_hessian(const Vector<IndexT, 3>&   flag,
                                               const Eigen::Vector<T, 3>& p,
                                               const Eigen::Vector<T, 3>& e0,
                                               const Eigen::Vector<T, 3>& e1,
                                               Eigen::Matrix<T, 9, 9>&    H)
{
    H.setZero();

    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {p, e0, e1};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_pe(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        Vector2i flag    = {1, 1};

        Eigen::Matrix<T, 6, 6> H6;
        point_point_distance2_hessian(flag, P0, P1, H6);

#pragma unroll
        for(int i = 0; i < 2; ++i)
            for(int j = 0; j < 2; ++j)
                H.template block<3, 3>(offsets[i] * 3, offsets[j] * 3) =
                    H6.template block<3, 3>(i * 3, j * 3);
    }
    else if(dim == 3)
    {
        details::H_PE3D(p[0], p[1], p[2], e0[0], e0[1], e0[2], e1[0], e1[1], e1[2], H.data());
    }
    else
    {
        MUDA_ERROR_WITH_LOCATION("Invalid flag (%d,%d,%d)", flag[0], flag[1], flag[2]);
    }
}

template <typename T>
MUDA_GENERIC void point_triangle_distance2_hessian(const Vector4i& flag,
                                                   const Eigen::Vector<T, 3>& p,
                                                   const Eigen::Vector<T, 3>& t0,
                                                   const Eigen::Vector<T, 3>& t1,
                                                   const Eigen::Vector<T, 3>& t2,
                                                   Eigen::Matrix<T, 12, 12>& H)
{
    H.setZero();

    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {p, t0, t1, t2};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_pt(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        Vector2i flag    = {1, 1};

        Eigen::Matrix<T, 6, 6> H6;
        point_point_distance2_hessian(flag, P0, P1, H6);

#pragma unroll
        for(int i = 0; i < 2; ++i)
            for(int j = 0; j < 2; ++j)
                H.template block<3, 3>(offsets[i] * 3, offsets[j] * 3) =
                    H6.template block<3, 3>(i * 3, j * 3);
    }
    else if(dim == 3)
    {
        Vector3i offsets = detail::pe_from_pt(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        auto&    P2      = P[offsets[2]];

        Eigen::Matrix<T, 9, 9> H9;
        point_edge_distance2_hessian(P0, P1, P2, H9);

#pragma unroll
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                H.template block<3, 3>(offsets[i] * 3, offsets[j] * 3) =
                    H9.template block<3, 3>(i * 3, j * 3);
    }
    else if(dim == 4)
    {
        details::H_PT(
            p[0], p[1], p[2], t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], H.data());
    }
    else
    {
        MUDA_ERROR_WITH_LOCATION(
            "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
    }
}

template <typename T>
MUDA_GENERIC void edge_edge_distance2_hessian(const Vector4i&            flag,
                                              const Eigen::Vector<T, 3>& ea0,
                                              const Eigen::Vector<T, 3>& ea1,
                                              const Eigen::Vector<T, 3>& eb0,
                                              const Eigen::Vector<T, 3>& eb1,
                                              Eigen::Matrix<T, 12, 12>&  H)
{
    H.setZero();

    IndexT              dim = detail::active_count(flag);
    Eigen::Vector<T, 3> P[] = {ea0, ea1, eb0, eb1};

    if(dim == 2)
    {
        Vector2i offsets = detail::pp_from_ee(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        Vector2i flag    = {1, 1};

        Eigen::Matrix<T, 6, 6> H6;
        point_point_distance2_hessian(flag, P0, P1, H6);

#pragma unroll
        for(int i = 0; i < 2; ++i)
            for(int j = 0; j < 2; ++j)
                H.template block<3, 3>(offsets[i] * 3, offsets[j] * 3) =
                    H6.template block<3, 3>(i * 3, j * 3);
    }
    else if(dim == 3)
    {
        Vector3i offsets = detail::pe_from_ee(flag);
        auto&    P0      = P[offsets[0]];
        auto&    P1      = P[offsets[1]];
        auto&    P2      = P[offsets[2]];

        Eigen::Matrix<T, 9, 9> H9;
        point_edge_distance2_hessian(P0, P1, P2, H9);

#pragma unroll
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                H.template block<3, 3>(offsets[i] * 3, offsets[j] * 3) =
                    H9.template block<3, 3>(i * 3, j * 3);
    }
    else if(dim == 4)
    {
        details::H_EE(ea0[0],
                      ea0[1],
                      ea0[2],
                      ea1[0],
                      ea1[1],
                      ea1[2],
                      eb0[0],
                      eb0[1],
                      eb0[2],
                      eb1[0],
                      eb1[1],
                      eb1[2],
                      H.data());
    }
    else
    {
        MUDA_ERROR_WITH_LOCATION(
            "Invalid flag (%d,%d,%d,%d)", flag[0], flag[1], flag[2], flag[3]);
    }
}
}  // namespace uipc::backend::cuda::distance
