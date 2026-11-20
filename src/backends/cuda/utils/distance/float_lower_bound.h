/*
 * NVIDIA contributions: FP32 distance lower bounds for broad-phase rejection.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

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
#include <type_define.h>

// Cheap FP32 rejection before the exact FP64 point-triangle and edge-edge
// distance tests. Each function evaluates a lower bound of the distance that
// is exact in real arithmetic (a 1-Lipschitz projection onto two orthonormal
// axes). It reports "farther" only when that bound exceeds `dist` by a margin
// far above FP32 rounding; poorly conditioned (near-degenerate or
// near-parallel) configurations never report "farther" and fall through to
// the exact test.
namespace uipc::backend::cuda::distance
{
namespace detail
{
    // Coordinates are made relative in FP64 before rounding, so every input
    // carries at most half an ulp of its local magnitude `scale`.
    inline UIPC_GENERIC bool clearly_farther(float lower_bound, Float dist, float scale)
    {
        return lower_bound > static_cast<float>(dist) * 1.001f + 1e-4f * scale;
    }

    inline UIPC_GENERIC float max_abs(const Eigen::Vector3f& a)
    {
        return a.cwiseAbs().maxCoeff();
    }

    // Gap between the closed intervals [a_lo, a_hi] and [b_lo, b_hi].
    inline UIPC_GENERIC float interval_gap(float a_lo, float a_hi, float b_lo, float b_hi)
    {
        return fmaxf(0.0f, fmaxf(b_lo - a_hi, a_lo - b_hi));
    }
}  // namespace detail

// True only if dist(p, triangle t0 t1 t2) > dist. The triangle lies in its
// plane and inside each edge's in-plane half-plane, so the distance is at
// least sqrt(h^2 + s^2): h to the plane, s outside the farthest edge.
inline UIPC_GENERIC bool point_triangle_farther_than(
    const Vector3& p, const Vector3& t0, const Vector3& t1, const Vector3& t2, Float dist)
{
    const Eigen::Vector3f f0 = (t0 - p).cast<float>();
    const Eigen::Vector3f f1 = (t1 - p).cast<float>();
    const Eigen::Vector3f f2 = (t2 - p).cast<float>();
    const float           scale =
        fmaxf(detail::max_abs(f0), fmaxf(detail::max_abs(f1), detail::max_abs(f2)));
    if(!(scale > 0.0f))  // every point coincides: exact test only
        return false;

    const Eigen::Vector3f n  = (f1 - f0).cross(f2 - f0);
    const float           nn = n.squaredNorm();
    const float           s2 = scale * scale;
    // |n| >= 0.1 scale^2 keeps the rounding of h and s near 100 ulp of
    // `scale`, two orders below the rejection margin.
    if(!(nn >= 1e-2f * s2 * s2))  // sliver or degenerate: exact test only
        return false;

    const float inv_n = (1.0f / sqrtf(nn));
    const float h     = fabsf(n.dot(f0)) * inv_n;

    // Outward in-plane edge normals m = e x n; the origin (the point) lies
    // outside edge (a, b) by -m.a / |m|.
    float                  s    = 0.0f;
    const Eigen::Vector3f* v[3] = {&f0, &f1, &f2};
    for(int e = 0; e < 3; ++e)
    {
        const Eigen::Vector3f& a = *v[e];
        const Eigen::Vector3f& b = *v[(e + 1) % 3];
        const Eigen::Vector3f  m = (b - a).cross(n);
        s = fmaxf(s, -m.dot(a) * (1.0f / sqrtf(m.squaredNorm())));
    }
    return detail::clearly_farther(sqrtf(h * h + s * s), dist, scale);
}

// True only if dist(segment ea, segment eb) > dist. Projecting onto the
// common normal n and one edge direction maps both segments to parallel
// planar segments whose distance sqrt(h^2 + gap^2) bounds the 3D distance.
inline UIPC_GENERIC bool edge_edge_farther_than(const Vector3& ea0,
                                                const Vector3& ea1,
                                                const Vector3& eb0,
                                                const Vector3& eb1,
                                                Float          dist)
{
    const Eigen::Vector3f a1 = (ea1 - ea0).cast<float>();
    const Eigen::Vector3f b0 = (eb0 - ea0).cast<float>();
    const Eigen::Vector3f b1 = (eb1 - ea0).cast<float>();
    const float           scale =
        fmaxf(detail::max_abs(a1), fmaxf(detail::max_abs(b0), detail::max_abs(b1)));
    if(!(scale > 0.0f))  // every end point coincides: exact test only
        return false;

    const Eigen::Vector3f u  = a1;
    const Eigen::Vector3f v  = b1 - b0;
    const float           uu = u.squaredNorm();
    const float           vv = v.squaredNorm();
    const float           s2 = scale * scale;
    if(!(uu >= 1e-6f * s2) || !(vv >= 1e-6f * s2))  // degenerate edge
        return false;

    const float inv_u = (1.0f / sqrtf(uu));
    const float pu0   = b0.dot(u) * inv_u;
    const float pu1   = b1.dot(u) * inv_u;
    const float g_u =
        detail::interval_gap(0.0f, uu * inv_u, fminf(pu0, pu1), fmaxf(pu0, pu1));

    const float inv_v = (1.0f / sqrtf(vv));
    const float pva   = u.dot(v) * inv_v;
    const float pvb   = b0.dot(v) * inv_v;
    const float g_v =
        detail::interval_gap(fminf(0.0f, pva), fmaxf(0.0f, pva), pvb, pvb + vv * inv_v);

    const Eigen::Vector3f n  = u.cross(v);
    const float           nn = n.squaredNorm();
    float                 h  = 0.0f;
    if(nn >= 1e-2f * uu * vv)  // common normal only for angles above ~6 degrees
        h = fabsf(n.dot(b0)) * (1.0f / sqrtf(nn));

    const float lower = sqrtf(h * h + fmaxf(g_u * g_u, g_v * g_v));
    return detail::clearly_farther(lower, dist, scale);
}
}  // namespace uipc::backend::cuda::distance
