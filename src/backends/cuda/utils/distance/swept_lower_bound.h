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
#include <utils/distance/float_lower_bound.h>

// Conservative rejection for the trajectory (swept) broad phase.
//
// Vertices move linearly, x(t) = x + t*dx with t in [0, 1] (dx already scaled by
// alpha). A point on primitive A and a point on primitive B with fixed
// barycentric weights move by convex combinations of their vertices'
// displacements, so their separation changes by at most
//   M = max_{a in A, b in B} |dx_a - dx_b|.
// Hence dist_t(A, B) >= dist_0(A, B) - M for every t, and a pair whose start
// distance exceeds `dist` + M can never come within `dist` during the step.
// Co-moving primitives (e.g. a body riding a conveyor) get M close to 0.
// The start-distance test reuses the FP32 bounds of float_lower_bound.h, which
// only report "farther" with a margin far above FP32 rounding.
//
// The rejected pair must also leave filter_toi unchanged. ACCD (ccd.inl) returns
// "no hit" right away when its first step bound (1 - eta)(d0 - thickness) /
// maxDispMag exceeds the initial toc of 1.1, where maxDispMag measures the
// displacements relative to their mean. Requiring d0 > thickness + kAccd *
// maxDispMag with kAccd = 1.1 / (1 - 0.1) * 1.005 covers every eta <= 0.1
// (ipc: 0.1, al-ipc: 0.001), so rejected pairs would have had toi = 1.1 anyway.
namespace uipc::backend::cuda::distance
{
namespace detail
{
    constexpr Float kAccd = 1.1 / (1.0 - 0.1) * 1.005;

    // Upper bound of ACCD's maxDispMag: max_i |da_i - mov| + max_j |db_j - mov|
    // with mov the mean displacement, inflated above FP32 rounding.
    template <int NA, int NB>
    inline UIPC_GENERIC Float accd_disp_mag(const Vector3* da, const Vector3* db)
    {
        Eigen::Vector3f mov = Eigen::Vector3f::Zero();
        float           mag = 0.0f;
        for(int i = 0; i < NA; ++i)
            mov += da[i].cast<float>();
        for(int j = 0; j < NB; ++j)
            mov += db[j].cast<float>();
        mov *= 1.0f / (NA + NB);
        float ma = 0.0f, mb = 0.0f;
        for(int i = 0; i < NA; ++i)
        {
            const Eigen::Vector3f a = da[i].cast<float>();
            mag                     = fmaxf(mag, max_abs(a));
            ma                      = fmaxf(ma, (a - mov).squaredNorm());
        }
        for(int j = 0; j < NB; ++j)
        {
            const Eigen::Vector3f b = db[j].cast<float>();
            mag                     = fmaxf(mag, max_abs(b));
            mb                      = fmaxf(mb, (b - mov).squaredNorm());
        }
        return static_cast<Float>((sqrtf(ma) + sqrtf(mb)) * 1.001f + 1e-5f * mag);
    }

    // Separating-axis lower bound: project both primitives onto an approximate
    // closest direction w and take the gap of the projected intervals. Any unit
    // axis gives a lower bound of the distance; a poor w only loosens it.
    // Coordinates are relative to one vertex (FP64 difference, then FP32), so the
    // rounding of the projections stays below 1e-6 * scale; the caller's margin
    // (dist * 1.001 + 1e-4 * scale) is two orders above that.
    inline UIPC_GENERIC float projected_gap(const Eigen::Vector3f& w,
                                            const Eigen::Vector3f* a,
                                            int                    na,
                                            const Eigen::Vector3f* b,
                                            int                    nb)
    {
        const float ww = w.squaredNorm();
        if(!(ww > 0.0f))
            return 0.0f;
        const Eigen::Vector3f u    = w * (1.0f / sqrtf(ww));
        float                 a_lo = u.dot(a[0]), a_hi = a_lo;
        float                 b_lo = u.dot(b[0]), b_hi = b_lo;
        for(int i = 1; i < na; ++i)
        {
            const float t = u.dot(a[i]);
            a_lo          = fminf(a_lo, t);
            a_hi          = fmaxf(a_hi, t);
        }
        for(int j = 1; j < nb; ++j)
        {
            const float t = u.dot(b[j]);
            b_lo          = fminf(b_lo, t);
            b_hi          = fmaxf(b_hi, t);
        }
        // |u| = 1 within a few ulp; shrink the gap by that relative error too.
        return interval_gap(a_lo, a_hi, b_lo, b_hi) * (1.0f - 1e-5f);
    }

    // Approximate closest points of segments p0p1, q0q1 (FP32, clamped).
    inline UIPC_GENERIC Eigen::Vector3f segment_closest_dir(const Eigen::Vector3f& p0,
                                                            const Eigen::Vector3f& p1,
                                                            const Eigen::Vector3f& q0,
                                                            const Eigen::Vector3f& q1)
    {
        const Eigen::Vector3f d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
        const float a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
        const float c = d1.dot(r), b = d1.dot(d2), den = a * e - b * b;
        float       s = (den > 1e-12f * a * e && den > 0.0f) ?
                            fminf(fmaxf((b * f - c * e) / den, 0.0f), 1.0f) :
                            0.0f;
        float       t = e > 0.0f ? (b * s + f) / e : 0.0f;
        if(t < 0.0f)
        {
            t = 0.0f;
            s = a > 0.0f ? fminf(fmaxf(-c / a, 0.0f), 1.0f) : 0.0f;
        }
        else if(t > 1.0f)
        {
            t = 1.0f;
            s = a > 0.0f ? fminf(fmaxf((b - c) / a, 0.0f), 1.0f) : 0.0f;
        }
        return (p0 + d1 * s) - (q0 + d2 * t);
    }

    // Approximate closest point of triangle t0t1t2 to p (Ericson, FP32).
    inline UIPC_GENERIC Eigen::Vector3f triangle_closest_dir(const Eigen::Vector3f& p,
                                                             const Eigen::Vector3f& a,
                                                             const Eigen::Vector3f& b,
                                                             const Eigen::Vector3f& c)
    {
        const Eigen::Vector3f ab = b - a, ac = c - a, ap = p - a;
        const float           d1 = ab.dot(ap), d2 = ac.dot(ap);
        if(d1 <= 0.0f && d2 <= 0.0f)
            return p - a;
        const Eigen::Vector3f bp = p - b;
        const float           d3 = ab.dot(bp), d4 = ac.dot(bp);
        if(d3 >= 0.0f && d4 <= d3)
            return p - b;
        // The denominators are |ab|^2, |ac|^2 and |bc|^2: a zero-length edge
        // gives no direction (projected_gap then bounds nothing).
        const float vc = d1 * d4 - d3 * d2;
        if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            return d1 - d3 > 0.0f ? Eigen::Vector3f(p - (a + ab * (d1 / (d1 - d3)))) :
                                    Eigen::Vector3f::Zero();
        const Eigen::Vector3f cp = p - c;
        const float           d5 = ab.dot(cp), d6 = ac.dot(cp);
        if(d6 >= 0.0f && d5 <= d6)
            return p - c;
        const float vb = d5 * d2 - d1 * d6;
        if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            return d2 - d6 > 0.0f ? Eigen::Vector3f(p - (a + ac * (d2 / (d2 - d6)))) :
                                    Eigen::Vector3f::Zero();
        const float va = d3 * d6 - d5 * d4;
        if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            return (d4 - d3) + (d5 - d6) > 0.0f ?
                       Eigen::Vector3f(
                           p - (b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))))) :
                       Eigen::Vector3f::Zero();
        const float den = va + vb + vc;
        if(!(den != 0.0f))
            return Eigen::Vector3f::Zero();
        return p - (a + ab * (vb / den) + ac * (vc / den));
    }

    // Upper bound of max |da_i - db_j|, inflated well above FP32 rounding of
    // the FP64 -> FP32 conversion and the norm.
    template <int NA, int NB>
    inline UIPC_GENERIC Float max_relative_motion(const Vector3* da, const Vector3* db)
    {
        float m2  = 0.0f;
        float mag = 0.0f;
        for(int i = 0; i < NA; ++i)
        {
            const Eigen::Vector3f a = da[i].cast<float>();
            mag                     = fmaxf(mag, max_abs(a));
            for(int j = 0; j < NB; ++j)
            {
                const Eigen::Vector3f b = db[j].cast<float>();
                mag                     = fmaxf(mag, max_abs(b));
                m2                      = fmaxf(m2, (a - b).squaredNorm());
            }
        }
        return static_cast<Float>(sqrtf(m2) * 1.001f + 1e-5f * mag);
    }
}  // namespace detail

// True only if edges ea, eb (moving by da*, db*) stay farther than `dist`
// apart for the whole step.
inline UIPC_GENERIC bool edge_edge_swept_farther_than(const Vector3& ea0,
                                                      const Vector3& ea1,
                                                      const Vector3& eb0,
                                                      const Vector3& eb1,
                                                      const Vector3& dea0,
                                                      const Vector3& dea1,
                                                      const Vector3& deb0,
                                                      const Vector3& deb1,
                                                      Float          dist,
                                                      Float          thickness)
{
    const Vector3 da[2] = {dea0, dea1};
    const Vector3 db[2] = {deb0, deb1};
    const Float   need =
        fmax(dist + detail::max_relative_motion<2, 2>(da, db),
             thickness + detail::kAccd * detail::accd_disp_mag<2, 2>(da, db));
    if(edge_edge_farther_than(ea0, ea1, eb0, eb1, need))
        return true;
    // Separating-axis bound along the approximate closest direction; it also
    // covers near-parallel pairs that float_lower_bound leaves to the exact test.
    const Eigen::Vector3f a[2] = {Eigen::Vector3f::Zero(), (ea1 - ea0).cast<float>()};
    const Eigen::Vector3f b[2] = {(eb0 - ea0).cast<float>(), (eb1 - ea0).cast<float>()};
    const float scale = fmaxf(detail::max_abs(a[1]),
                              fmaxf(detail::max_abs(b[0]), detail::max_abs(b[1])));
    const float gap   = detail::projected_gap(
        detail::segment_closest_dir(a[0], a[1], b[0], b[1]), a, 2, b, 2);
    return detail::clearly_farther(gap, need, scale);
}

// True only if point p (moving by dp) and triangle t (moving by dt*) stay
// farther than `dist` apart for the whole step.
inline UIPC_GENERIC bool point_triangle_swept_farther_than(const Vector3& p,
                                                           const Vector3& t0,
                                                           const Vector3& t1,
                                                           const Vector3& t2,
                                                           const Vector3& dp,
                                                           const Vector3& dt0,
                                                           const Vector3& dt1,
                                                           const Vector3& dt2,
                                                           Float          dist,
                                                           Float thickness)
{
    const Vector3 da[1] = {dp};
    const Vector3 db[3] = {dt0, dt1, dt2};
    const Float   need =
        fmax(dist + detail::max_relative_motion<1, 3>(da, db),
             thickness + detail::kAccd * detail::accd_disp_mag<1, 3>(da, db));
    if(point_triangle_farther_than(p, t0, t1, t2, need))
        return true;
    // Separating-axis bound; it also covers slivers that float_lower_bound
    // leaves to the exact test.
    const Eigen::Vector3f a[1] = {Eigen::Vector3f::Zero()};
    const Eigen::Vector3f b[3] = {
        (t0 - p).cast<float>(), (t1 - p).cast<float>(), (t2 - p).cast<float>()};
    const float scale = fmaxf(detail::max_abs(b[0]),
                              fmaxf(detail::max_abs(b[1]), detail::max_abs(b[2])));
    const float gap   = detail::projected_gap(
        detail::triangle_closest_dir(a[0], b[0], b[1], b[2]), a, 1, b, 3);
    return detail::clearly_farther(gap, need, scale);
}
}  // namespace uipc::backend::cuda::distance
