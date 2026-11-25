/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
// Helpers for joint Hessian kernels that give one warp to one joint (see
// warp_spd.h for why a warp per joint is cheaper than a thread per joint).

// Block size of such kernels: one warp, one joint per block.
constexpr int JointWarpSize = 32;

// Entry (a, b) of J_l^T H_F J_r for the "two frames per body" joint maps
// F = J q used by the revolute joint limit and the external articulation
// constraints:
//   F(6 * body + r)     = sum_c basis_body(c)     * q_body(3 + 3 r + c)
//                         [+ q_body(r) when Translation]
//   F(6 * body + 3 + r) = sum_c basis_body(3 + c) * q_body(3 + 3 r + c)
// with basis_body = (bk, bl) on the left and (bm, bn) on the right, r, c in
// [0, 3). H_F is 12 x 12 column-major (shared memory); a, b index the 24
// generalized coordinates of the two bodies. Without Translation the
// translation coordinates q(0..2) do not enter F (zero rows and columns).
namespace warp_joint_hessian_detail
{
    // The (at most two) F entries coordinate a enters, with their weights.
    template <bool Translation>
    UIPC_DEVICE inline int f_terms(const Vector6& b0, const Vector6& b1, int a, int* f, Float* w)
    {
        const int      body = a / 12;
        const int      ia   = a % 12;
        const Vector6& base = body == 0 ? b0 : b1;
        if(ia < 3)
        {
            if constexpr(Translation)
            {
                f[0] = 6 * body + ia;
                w[0] = 1;
                return 1;
            }
            return 0;
        }
        const int r = (ia - 3) / 3, c = (ia - 3) % 3;
        f[0] = 6 * body + r;
        w[0] = base(c);
        f[1] = 6 * body + 3 + r;
        w[1] = base(3 + c);
        return 2;
    }
}  // namespace warp_joint_hessian_detail

template <bool Translation = false>
UIPC_DEVICE inline Float basis_jt_h_j_entry(const Float*   H_F,
                                            const Vector6& bk,
                                            const Vector6& bl,
                                            const Vector6& bm,
                                            const Vector6& bn,
                                            int            a,
                                            int            b)
{
    int   fa[2], fb[2];
    Float wa[2], wb[2];
    const int na = warp_joint_hessian_detail::f_terms<Translation>(bk, bl, a, fa, wa);
    const int nb = warp_joint_hessian_detail::f_terms<Translation>(bm, bn, b, fb, wb);
    Float result = 0;
    for(int j = 0; j < nb; ++j)
    {
        // (sum_i wa_i H(fa_i, fb_j)) * wb_j
        Float x = 0;
        for(int i = 0; i < na; ++i)
            x += wa[i] * H_F[fa[i] + 12 * fb[j]];
        result += wb[j] * x;
    }
    return result;
}

// Entry (a, b) of J01^T H01 J01 for the prismatic joint frame map F01 = J01 q
// (affine_body_prismatic_joint and, with SignTj = -1, the driving prismatic
// joint / prismatic joint limit map):
//   F01(r)     = q_j(r) - q_i(r) + sum_c (cj(c) q_j - ci(c) q_i)(3 + 3 r + c)
//   F01(3 + r) = sum_c ti(c) q_i(3 + 3 r + c)
//   F01(6 + r) = SignTj sum_c tj(c) q_j(3 + 3 r + c)
// H01 is 9 x 9 column-major (shared memory).
namespace warp_joint_hessian_detail
{
    template <int SignTj>
    UIPC_DEVICE inline int prismatic_f_terms(const Vector3& ci,
                                             const Vector3& ti,
                                             const Vector3& cj,
                                             const Vector3& tj,
                                             int            a,
                                             int*           f,
                                             Float*         w)
    {
        const int body = a / 12;
        const int ia   = a % 12;
        if(ia < 3)
        {
            f[0] = ia;
            w[0] = body == 0 ? Float(-1) : Float(1);
            return 1;
        }
        const int r = (ia - 3) / 3, c = (ia - 3) % 3;
        f[0] = r;
        w[0] = body == 0 ? -ci(c) : cj(c);
        f[1] = body == 0 ? 3 + r : 6 + r;
        w[1] = body == 0 ? ti(c) : SignTj * tj(c);
        return 2;
    }
}  // namespace warp_joint_hessian_detail

template <int SignTj>
UIPC_DEVICE inline Float prismatic_jt_h_j_entry(const Float*   H01,
                                                const Vector3& ci,
                                                const Vector3& ti,
                                                const Vector3& cj,
                                                const Vector3& tj,
                                                int            a,
                                                int            b)
{
    int       fa[2], fb[2];
    Float     wa[2], wb[2];
    const int na =
        warp_joint_hessian_detail::prismatic_f_terms<SignTj>(ci, ti, cj, tj, a, fa, wa);
    const int nb =
        warp_joint_hessian_detail::prismatic_f_terms<SignTj>(ci, ti, cj, tj, b, fb, wb);
    Float result = 0;
    for(int j = 0; j < nb; ++j)
    {
        Float x = 0;
        for(int i = 0; i < na; ++i)
            x += wa[i] * H01[fa[i] + 9 * fb[j]];
        result += wb[j] * x;
    }
    return result;
}

// Entry (a, b) of the (q-independent) Hessian of the prismatic joint's
// orientation term E23 = kappa / 2 sum_{u in {n, b}} sum_r
// (u_i . q_i,row r - u_j . q_j,row r)^2, row r = q(3 + 3 r .. 5 + 3 r).
UIPC_DEVICE inline Float prismatic_e23_hessian_entry(Float          kappa,
                                                     const Vector3& ni,
                                                     const Vector3& bi,
                                                     const Vector3& nj,
                                                     const Vector3& bj,
                                                     int            a,
                                                     int            b)
{
    const int ia = a % 12, ib = b % 12;
    if(ia < 3 || ib < 3 || (ia - 3) / 3 != (ib - 3) / 3)
        return 0;
    const int      ca = (ia - 3) % 3, cb = (ib - 3) % 3;
    const bool     ja = a >= 12, jb = b >= 12;
    const Vector3& na = ja ? nj : ni;
    const Vector3& ba = ja ? bj : bi;
    const Vector3& nb = jb ? nj : ni;
    const Vector3& bb = jb ? bj : bi;
    const Float    v  = kappa * (na(ca) * nb(cb) + ba(ca) * bb(cb));
    return ja == jb ? v : -v;
}

// Warp-cooperative equivalent of
//   TripletMatrixAssembler{T}.half_block<2>(I).write(l_indices, r_indices, H)
// for a 24x24 H in shared memory (column-major): the same three triplets
// (blocks (0,0), (0,1), (1,1), each placed in the upper triangle of the global
// matrix exactly as upper_LR() does) at triplets I, I + 1, I + 2; lanes write
// different coefficients. Every lane of the warp must call it.
template <typename TripletView>
UIPC_DEVICE void warp_write_half_block2(const TripletView& T,
                                        int                I,
                                        const Vector2i&    l_indices,
                                        const Vector2i&    r_indices,
                                        const Float*       H,
                                        int                lane)
{
    const int2 off = T.submatrix_offset();
#pragma unroll
    for(int t = 0; t < 3; ++t)
    {
        const int  ii   = t == 2 ? 1 : 0;
        const int  jj   = t == 0 ? 0 : 1;
        const bool keep = l_indices(ii) + off.x < r_indices(jj) + off.y;
        const int  L    = keep ? ii : jj;
        const int  R    = keep ? jj : ii;
        if(lane == 0)
        {
            T.row_indices()(I + t) = off.x + l_indices(L);
            T.col_indices()(I + t) = off.y + r_indices(R);
        }
        Float* value = T.values()(I + t).data();
        for(int e = lane; e < 144; e += 32)
        {
            const int row = e % 12;
            const int col = e / 12;
            value[e]      = H[(12 * L + row) + 24 * (12 * R + col)];
        }
    }
}
}  // namespace uipc::backend::cuda
