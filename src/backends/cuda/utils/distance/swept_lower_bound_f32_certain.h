/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <utils/distance/swept_lower_bound_f32.h>

// Tests that hold only where edge_edge_swept_farther_than_f32 rejects in its
// first test (edge_edge_bound_f32's bound above clearly_farther_f32's
// threshold), or in its second (projected_gap along segment_closest_dir above
// the same threshold), under every FP32 evaluation of its source, however the
// compiler fuses multiplies into adds. The stored-pair judge uses them to skip
// the EE predicate for most pairs while every flag it keeps still comes from
// the predicate's own code.
//
// Model (nvcc without fast math): every operation is correctly rounded (unit
// roundoff u = 2^-24; sqrtf and division too), fminf, fmaxf, fabsf and
// negation are exact, the expression tree is the source's, and an evaluation
// may fuse a multiply into the add or subtract that consumes it. So:
//  (a) a value made from loaded inputs by one rounding or exact operations is
//      the same in every evaluation: the coordinates relative to ea0 (a1, b0,
//      b1), v = b1 - b0, scale, s2 and 1e-6 s2, rounding (2^-20 times a max),
//      thickness, dist ((w0 + w1) * 0.5 is exact, so fusing it into the next
//      add changes nothing), the step differences and their mean (adds in
//      source order, then times 1/4) and mag;
//  (b) a sum of k products, in any order and with or without fused
//      multiply-adds, is within gamma_k = k u / (1 - k u) times the sum of the
//      products' magnitudes of its real value (Higham, sec. 3.1);
//  (c) sqrt and division add one rounding each.
// Products of tiny components may underflow, with an absolute error of at most
// 2^-149 each; kTiny (1e-35) in the absolute bounds and the relative margins
// against quantities of at least 1e-16 (scale >= 2^-16) cover that.
//
// Threshold T = need * 1.001 + 1e-4 scale + rounding. Its terms are
// nonnegative (thickness, dist >= 0 are required), so each node's relative
// error is at most its operands' largest plus u: squared norms gamma_3, roots
// 2.5u, motion 4.5u, accd 5.5u, dist + motion 5.5u, thickness + kAccd accd
// 7.5u, need 7.5u, T 10.5u. Any evaluation's T is thus at most the screen's
// times (1 + 10.5u) / (1 - 10.5u) < 1 + 22u, below T_up = T (1 + kSafe)
// (kSafe = 4e-6 > 66u).
//
// Bound. The conditioning tests compare uu, vv (within gamma_3) with the same
// 1e-6 s2 in every evaluation, so uu (1 - kSafe) >= 1e-6 s2 passes them in all.
// With real values from exact arithmetic on the values of (a), these bound
// |computed - real| in any evaluation: 1/sqrt(uu) 3.5u relative; a projection
// (x.u) / |u| (gamma_3 + 4.5u) A / |u| < 7.6u A / |u|, A = sum |x_k u_k|,
// taken as 12u A inv_u; |u| = uu inv_u 7.5u |u|, taken as 10u; a difference,
// fused or not, the operands' bounds plus u (|x| + |y|); min and max are
// 1-Lipschitz, so the gaps g_u and g_v inherit their arguments' bounds; the
// normal n = u x v gamma_2 C_k per component (C_k the magnitudes of its two
// products). The screen's g_u, g_v thus differ from any evaluation's by at
// most twice those bounds. h counts only if nn >= 1e-2 uu vv holds in every
// evaluation (a lower bound of nn above an upper bound of the threshold); then
// a lower bound of |n.b0| over an upper bound of sqrt(nn) bounds it below.
// Otherwise h is taken as 0, a valid lower bound: sqrt(h^2 + M) rounds to at
// least sqrt(M). LB_low = sqrt(h_low^2 + max(gu_low^2, gv_low^2)) (1 - kSafe)
// is then below every evaluation's bound (squares, sum and root lose under
// 3u), and LB_low > T_up makes every evaluation's bound exceed its threshold.
//
// Gap. segment_closest_dir(0, a1, b0, b1) computes s (a clamped quotient num /
// den, or 0 when den fails its test), then t = (b s + f) / e, and replaces s
// when t is clamped; the direction is w = a1 s - (b0 + v t). Each evaluation
// takes one of three branches, and its w lies in that branch's set:
//  - t kept (0 <= t <= 1): t is within 17.3u X / |v| of (v.(a1 s - b0)) / |v|^2
//    (X >= |a1|, |b0|, |b1|), so w is within 27.3u X of perp_v(a1 s - b0), the
//    part of a1 s - b0 normal to v, which is affine in s; s lies in [0, 1], or
//    in a narrower range when den passes its test in every evaluation (den and
//    num within 16u a e and 16.1u sqrt(a) e X of their real values);
//  - t < 0: w is within 3u X of a1 s - b0 for s in the clamped range of
//    -c / a (within 4.01u |q| + 3.02u sum |a1_k b0_k| / a of its real value);
//  - t > 1: likewise w = a1 s - (b0 + v) for s from (b - c) / a.
// A branch is left out when the range of t (affine in s; within 3.02u (sum
// |a1_k v_k| + sum |v_k b0_k|) / e + 1.01u |b| / e + 5.1u |t| of its real
// value) excludes it in every evaluation. Along a set w in [wa, wb] grown by
// rho, projected_gap's two interval gaps times |w|, C1(w) = min(w.b0, w.b1)
// - max(0, w.a1) and C2(w) = min(0, w.a1) - max(w.b0, w.b1), are concave, so
// they are least at the segment's ends, and moving w by rho moves them by at
// most 2 rho X; |w| is at most the larger end's plus rho. The evaluation's
// gap, along its rounded unit direction (3.5u relative, u per component) with
// rounded projections (gamma_3), is then at least (1 - 4.6u) times the least
// max(C1, C2) / |w| minus 8.1u X, times its (1 - 1e-5) factor and a rounding;
// the screen's own errors (each set's ends, the dots of C1 and C2) are inside
// the margins: rho 64u X for t kept and 8u X for the clamped branches.
// Ranges: scale in [2^-16, 2^16] and mag, dist, rounding <= 2^16 (nothing
// overflows), and both edges conditioned as for the first test; otherwise, or
// with NaN inputs, the tests return false.
namespace uipc::backend::cuda::distance
{
namespace detail
{
    constexpr float kCertainU = 0x1p-24f;
    constexpr float kCertainSafe = 4e-6f;  // > 64 u, covers the relative factors above
    constexpr float kCertainTiny = 1e-35f;  // absolute slack for underflowed products

    inline UIPC_GENERIC float abs_dot(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
        return fabsf(a.x() * b.x()) + fabsf(a.y() * b.y()) + fabsf(a.z() * b.z());
    }

    inline UIPC_GENERIC bool certain_range(float x)
    {
        return x >= 0x1p-16f && x <= 0x1p16f;
    }

    // What both tests share: the coordinates relative to ea0 and an upper
    // bound of every evaluation's threshold.
    struct CertainEdgeEdge
    {
        Eigen::Vector3f u, b0, b1, v;  // a1, b0, b1 relative to ea0; v = b1 - b0
        float uu, vv, T_up;
    };

    // False outside the model's ranges, or unless both edges pass
    // edge_edge_bound_f32's conditioning in every evaluation.
    inline UIPC_GENERIC bool certain_edge_edge(const float4&    ea0,
                                               const float4&    ea1,
                                               const float4&    eb0,
                                               const float4&    eb1,
                                               const float4&    dea0,
                                               const float4&    dea1,
                                               const float4&    deb0,
                                               const float4&    deb1,
                                               float            skin,
                                               CertainEdgeEdge& c)
    {
        // the threshold, as edge_edge_swept_farther_than_f32 computes it
        const float           thickness = dea0.w + deb0.w;
        const float           dist  = (ea0.w + eb0.w) * 0.5f + thickness + skin;
        const Eigen::Vector3f da[2] = {xyz(dea0), xyz(dea1)};
        const Eigen::Vector3f db[2] = {xyz(deb0), xyz(deb1)};
        float                 motion, accd;
        motion_bounds_f32<2, 2>(da, db, motion, accd);
        const float need = swept_need_f32(dist, thickness, motion, accd);

        const Eigen::Vector3f o = xyz(ea0);
        c.u                     = xyz(ea1) - o;
        c.b0                    = xyz(eb0) - o;
        c.b1                    = xyz(eb1) - o;
        const float scale = fmaxf(max_abs(c.u), fmaxf(max_abs(c.b0), max_abs(c.b1)));
        const float rounding = 0x1p-20f
                               * fmaxf(fmaxf(max_abs(ea0), max_abs(ea1)),
                                       fmaxf(max_abs(eb0), max_abs(eb1)));
        const float mag = fmaxf(fmaxf(max_abs(da[0]), max_abs(da[1])),
                                fmaxf(max_abs(db[0]), max_abs(db[1])));
        if(!certain_range(scale) || !(mag <= 0x1p16f) || !(thickness >= 0.0f)
           || !(dist >= 0.0f) || !(dist <= 0x1p16f) || !(rounding <= 0x1p16f))
            return false;
        c.T_up = (need * 1.001f + 1e-4f * scale + rounding) * (1.0f + kCertainSafe);

        // edge_edge_bound_f32's conditioning, passed in every evaluation
        c.v             = c.b1 - c.b0;
        c.uu            = c.u.squaredNorm();
        c.vv            = c.v.squaredNorm();
        const float s2  = scale * scale;
        const float thr = 1e-6f * s2;
        return c.uu * (1.0f - kCertainSafe) >= thr
               && c.vv * (1.0f - kCertainSafe) >= thr && c.T_up < 0x1p60f;
    }

    // A lower bound of max(C1(w), C2(w)) / |w| over the directions w of the
    // segment [wa, wb] grown by rho, where C1(w) = min(w.b0, w.b1) - max(0,
    // w.a1) and C2(w) = min(0, w.a1) - max(w.b0, w.b1) are projected_gap's two
    // interval gaps (a0 = 0) times |w|. Both are concave, so over the segment
    // they are least at its ends, and moving w by rho moves them by at most
    // 2 rho X (X >= |a1|, |b0|, |b1|); |w| is at most the larger end's plus rho.
    inline UIPC_GENERIC float certain_gap_over(const Eigen::Vector3f& wa,
                                               const Eigen::Vector3f& wb,
                                               float                  rho,
                                               const Eigen::Vector3f& a1,
                                               const Eigen::Vector3f& b0,
                                               const Eigen::Vector3f& b1,
                                               float                  X)
    {
        constexpr float U  = kCertainU;
        float           c1 = 0.0f, c2 = 0.0f, wmax = 0.0f;
        for(int k = 0; k < 2; ++k)
        {
            const Eigen::Vector3f& w  = k == 0 ? wa : wb;
            const float            pa = w.dot(a1);
            const float            p0 = w.dot(b0);
            const float            p1 = w.dot(b1);
            const float            n  = sqrtf(w.squaredNorm());
            // the dots within 3.01u |w| X, the difference one rounding more
            const float e  = 12.0f * U * n * X + kCertainTiny;
            const float d1 = fminf(p0, p1) - fmaxf(0.0f, pa) - e;
            const float d2 = fminf(0.0f, pa) - fmaxf(p0, p1) - e;
            c1             = k == 0 ? d1 : fminf(c1, d1);
            c2             = k == 0 ? d2 : fminf(c2, d2);
            wmax           = fmaxf(wmax, n);
        }
        const float h = fmaxf(c1, c2) - 2.0f * rho * X;
        if(!(h > 0.0f))
            return 0.0f;
        return h / (wmax * (1.0f + 8.0f * U) + rho) * (1.0f - 8.0f * U);
    }
}  // namespace detail

// True only if edge_edge_swept_farther_than_f32 with these arguments returns
// true from its first test (edge_edge_bound_f32 above the threshold) under
// every FP32 evaluation (see above).
inline UIPC_GENERIC bool edge_edge_swept_farther_than_f32_certain_bound(
    const float4& ea0,
    const float4& ea1,
    const float4& eb0,
    const float4& eb1,
    const float4& dea0,
    const float4& dea1,
    const float4& deb0,
    const float4& deb1,
    float         skin)
{
    using detail::kCertainSafe;
    using detail::kCertainTiny;
    constexpr float         U = detail::kCertainU;
    detail::CertainEdgeEdge c;
    if(!detail::certain_edge_edge(ea0, ea1, eb0, eb1, dea0, dea1, deb0, deb1, skin, c))
        return false;
    const Eigen::Vector3f& u  = c.u;
    const Eigen::Vector3f& b0 = c.b0;
    const Eigen::Vector3f& b1 = c.b1;
    const Eigen::Vector3f& v  = c.v;
    const float            uu = c.uu;
    const float            vv = c.vv;

    // g_u: projections onto u.
    const float inv_u = 1.0f / sqrtf(uu);
    const float pu0   = b0.dot(u) * inv_u;
    const float pu1   = b1.dot(u) * inv_u;
    const float len_u = uu * inv_u;
    const float e_pu =
        12.0f * U * inv_u * fmaxf(detail::abs_dot(b0, u), detail::abs_dot(b1, u));
    const float e_lenu = 10.0f * U * len_u;
    const float g_u =
        detail::interval_gap(0.0f, len_u, fminf(pu0, pu1), fmaxf(pu0, pu1));
    const float e_gu =
        e_pu + e_lenu + 2.0f * U * (fmaxf(fabsf(pu0), fabsf(pu1)) + len_u) + kCertainTiny;

    // g_v: projections onto v.
    const float inv_v  = 1.0f / sqrtf(vv);
    const float pva    = u.dot(v) * inv_v;
    const float pvb    = b0.dot(v) * inv_v;
    const float len_v  = vv * inv_v;
    const float top    = pvb + len_v;
    const float e_pva  = 12.0f * U * inv_v * detail::abs_dot(u, v);
    const float e_pvb  = 12.0f * U * inv_v * detail::abs_dot(b0, v);
    const float e_lenv = 10.0f * U * len_v;
    const float e_top  = e_pvb + e_lenv + 2.0f * U * (fabsf(pvb) + len_v);
    const float g_v = detail::interval_gap(fminf(0.0f, pva), fmaxf(0.0f, pva), pvb, top);
    const float e_gv = fmaxf(e_pvb + e_pva + 2.0f * U * (fabsf(pvb) + fabsf(pva)),
                             e_pva + e_top + 2.0f * U * (fabsf(pva) + fabsf(top)))
                       + kCertainTiny;

    // h: the common normal, when every evaluation computes it.
    const Eigen::Vector3f n = u.cross(v);
    const Eigen::Vector3f cn{fabsf(u.y() * v.z()) + fabsf(u.z() * v.y()),
                             fabsf(u.z() * v.x()) + fabsf(u.x() * v.z()),
                             fabsf(u.x() * v.y()) + fabsf(u.y() * v.x())};
    // |n_k - real| <= gamma_2 C_k < 3u C_k for any evaluation; twice that
    // between the screen's and another evaluation's components
    const Eigen::Vector3f e_n{cn.x() * (6.0f * U) + kCertainTiny,
                              cn.y() * (6.0f * U) + kCertainTiny,
                              cn.z() * (6.0f * U) + kCertainTiny};
    float                 nn_low = 0.0f, nn_up = 0.0f;
    for(int k = 0; k < 3; ++k)
    {
        const float lo = fmaxf(0.0f, fabsf(n[k]) - e_n[k]);
        const float hi = fabsf(n[k]) + e_n[k];
        nn_low += lo * lo;
        nn_up += hi * hi;
    }
    nn_low *= 1.0f - kCertainSafe;
    nn_up *= 1.0f + kCertainSafe;
    const float thr_nn_up = 1e-2f * (uu * (1.0f + kCertainSafe))
                            * (vv * (1.0f + kCertainSafe)) * (1.0f + kCertainSafe);
    float h_low = 0.0f;
    if(nn_low >= thr_nn_up && nn_up > 0.0f)
    {
        const float dot = n.dot(b0);
        // the dot of a perturbed n with b0: the perturbations, then gamma_3
        const float e_dot = 2.0f
                                * (e_n.x() * fabsf(b0.x()) + e_n.y() * fabsf(b0.y())
                                   + e_n.z() * fabsf(b0.z()))
                            + 8.0f * U
                                  * ((fabsf(n.x()) + e_n.x()) * fabsf(b0.x())
                                     + (fabsf(n.y()) + e_n.y()) * fabsf(b0.y())
                                     + (fabsf(n.z()) + e_n.z()) * fabsf(b0.z()))
                            + kCertainTiny;
        h_low = fmaxf(0.0f, fabsf(dot) - e_dot) * (1.0f / sqrtf(nn_up)) * (1.0f - kCertainSafe);
    }

    const float gu_low = fmaxf(0.0f, g_u - 2.0f * e_gu);
    const float gv_low = fmaxf(0.0f, g_v - 2.0f * e_gv);
    const float LB_low = sqrtf(h_low * h_low + fmaxf(gu_low * gu_low, gv_low * gv_low))
                         * (1.0f - kCertainSafe);
    return LB_low > c.T_up;
}

// True only if edge_edge_swept_farther_than_f32 with these arguments rejects
// in its second test (projected_gap along segment_closest_dir above the
// threshold) under every FP32 evaluation (see above).
inline UIPC_GENERIC bool edge_edge_swept_farther_than_f32_certain_gap(
    const float4& ea0,
    const float4& ea1,
    const float4& eb0,
    const float4& eb1,
    const float4& dea0,
    const float4& dea1,
    const float4& deb0,
    const float4& deb1,
    float         skin)
{
    using detail::kCertainSafe;
    using detail::kCertainTiny;
    constexpr float         U = detail::kCertainU;
    detail::CertainEdgeEdge c;
    if(!detail::certain_edge_edge(ea0, ea1, eb0, eb1, dea0, dea1, deb0, deb1, skin, c))
        return false;
    const Eigen::Vector3f& u  = c.u;
    const Eigen::Vector3f& b0 = c.b0;
    const Eigen::Vector3f& b1 = c.b1;
    const Eigen::Vector3f& v  = c.v;
    const float            uu = c.uu;
    const float            vv = c.vv;
    const float X = sqrtf(fmaxf(uu, fmaxf(b0.squaredNorm(), b1.squaredNorm())))
                    * (1.0f + 8.0f * U);

    // segment_closest_dir(0, a1, b0, b1): a = uu, e = vv and
    const float bb   = u.dot(v);    // b = a1.v
    const float cc   = -u.dot(b0);  // c = a1.r, r = -b0
    const float ff   = -v.dot(b0);  // f = v.r
    const float A_av = detail::abs_dot(u, v);
    const float A_ab = detail::abs_dot(u, b0);
    const float A_vb = detail::abs_dot(v, b0);

    // s before t's clamps lies in [s_lo, s_hi] in every evaluation: the clamped
    // quotient num / den if den passes its test in every evaluation, else
    // anything in [0, 1]. den and num are within 16u a e and 16.1u sqrt(a) e X
    // of their real values in any evaluation.
    float       s_lo = 0.0f, s_hi = 1.0f;
    const float den   = uu * vv - bb * bb;
    const float e_den = 40.0f * U * uu * vv;
    if(den - e_den > 1e-12f * uu * vv * (1.0f + kCertainSafe))
    {
        const float num   = bb * ff - cc * vv;
        const float e_num = 40.0f * U * sqrtf(uu) * vv * X;
        const float d_lo  = den - e_den;
        const float d_hi  = den + e_den;
        const float n_lo  = num - e_num;
        const float n_hi  = num + e_num;
        const float q0 = n_lo / d_lo, q1 = n_lo / d_hi, q2 = n_hi / d_lo, q3 = n_hi / d_hi;
        float q_lo = fminf(fminf(q0, q1), fminf(q2, q3));
        float q_hi = fmaxf(fmaxf(q0, q1), fmaxf(q2, q3));
        q_lo -= 4.0f * U * fabsf(q_lo);
        q_hi += 4.0f * U * fabsf(q_hi);
        s_lo = fminf(fmaxf(q_lo, 0.0f), 1.0f);
        s_hi = fminf(fmaxf(q_hi, 0.0f), 1.0f);
    }

    // t = (b s + f) / e before its clamps lies in [t_lo, t_hi] in every
    // evaluation (affine in s; within 3.02u (A_av + A_vb) / e + 1.01u |b| / e
    // + 5.1u |t| of its real value).
    const float tA = (bb * s_lo + ff) / vv;
    const float tB = (bb * s_hi + ff) / vv;
    const float e_t =
        2.0f
        * (8.0f * U * fmaxf(fabsf(tA), fabsf(tB))
           + (4.0f * U * (A_av + A_vb) + 3.0f * U * (fabsf(bb) + fabsf(ff))) / vv);
    const float t_lo = fminf(tA, tB) - e_t;
    const float t_hi = fmaxf(tA, tB) + e_t;

    // Every evaluation's direction w lies in the set of the branch it takes;
    // g bounds the gap along every direction of every branch that may run.
    if(!(t_lo <= t_hi))
        return false;
    float g = 3.0e38f;
    if(t_hi >= 0.0f && t_lo <= 1.0f)
    {
        // t kept: w = perp_v(a1 s - b0) for s in [s_lo, s_hi], within 27.3u X
        // in the evaluation and 23u X in this one
        const float inv_vv = 1.0f / vv;
        auto        perp   = [&](float s)
        {
            const Eigen::Vector3f x = u * s - b0;
            return Eigen::Vector3f(x - v * (v.dot(x) * inv_vv));
        };
        g = fminf(g, detail::certain_gap_over(perp(s_lo), perp(s_hi), 64.0f * U * X, u, b0, b1, X));
    }
    if(t_lo < 0.0f)
    {
        // t = 0, s = clamp(-c / a): w = a1 s - b0, within 3u X each
        const float q  = -cc / uu;
        const float eq = 2.0f * (5.0f * U * fabsf(q) + 4.0f * U * A_ab / uu);
        const float sa = fminf(fmaxf(q - eq, 0.0f), 1.0f);
        const float sb = fminf(fmaxf(q + eq, 0.0f), 1.0f);
        g = fminf(g, detail::certain_gap_over(u * sa - b0, u * sb - b0, 8.0f * U * X, u, b0, b1, X));
    }
    if(t_hi > 1.0f)
    {
        // t = 1, s = clamp((b - c) / a): w = a1 s - (b0 + v), within 3u X each
        const Eigen::Vector3f B1 = b0 + v;
        const float           q  = (bb - cc) / uu;
        const float eq = 2.0f * (6.0f * U * fabsf(q) + 4.0f * U * (A_av + A_ab) / uu);
        const float sa = fminf(fmaxf(q - eq, 0.0f), 1.0f);
        const float sb = fminf(fmaxf(q + eq, 0.0f), 1.0f);
        g = fminf(g, detail::certain_gap_over(u * sa - B1, u * sb - B1, 8.0f * U * X, u, b0, b1, X));
    }

    // projected_gap's projections onto its rounded unit direction lose at most
    // 4.6u relative and 8.1u X; then its (1 - 1e-5) factor
    const float gap_low = (g * (1.0f - kCertainSafe) - 12.0f * U * X - kCertainTiny)
                          * (1.0f - 1e-5f) * (1.0f - kCertainSafe);
    return gap_low > c.T_up;
}

// True only if edge_edge_swept_farther_than_f32 with these arguments returns
// true under every FP32 evaluation: by its first test, or by its second.
inline UIPC_GENERIC bool edge_edge_swept_farther_than_f32_certain(const float4& ea0,
                                                                  const float4& ea1,
                                                                  const float4& eb0,
                                                                  const float4& eb1,
                                                                  const float4& dea0,
                                                                  const float4& dea1,
                                                                  const float4& deb0,
                                                                  const float4& deb1,
                                                                  float skin)
{
    return edge_edge_swept_farther_than_f32_certain_bound(
               ea0, ea1, eb0, eb1, dea0, dea1, deb0, deb1, skin)
           || edge_edge_swept_farther_than_f32_certain_gap(
               ea0, ea1, eb0, eb1, dea0, dea1, deb0, deb1, skin);
}
}  // namespace uipc::backend::cuda::distance
