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
#include <utils/distance/swept_lower_bound.h>

// The swept rejections of swept_lower_bound.h on FP32 inputs, so that a
// trajectory leaf test rejects most far pairs without any FP64 work.
//
// Inputs: per-vertex FP32 positions (x, y, z, d_hat) and FP32 step
// displacements (dx, dy, dz, thickness), each rounded once from FP64 per
// detect. The displacements equal the FP32 casts swept_lower_bound.h makes, so
// the motion bounds M and maxDispMag are the same. The positions are absolute:
// rounding them moves each coordinate relative to another vertex of the pair by
// at most 2 ulp of the pair's largest coordinate magnitude X (both roundings and
// the subtraction), each vertex by 2 sqrt(3) ulp(X) and the distance between
// the two primitives by 4 sqrt(3) ulp(X) < 2^-20 X. The rejection margin adds
// that to swept_lower_bound.h's, so a rejected pair still satisfies both of its
// conditions: it stays farther than `dist` apart and ACCD reports no hit.
namespace uipc::backend::cuda::distance
{
namespace detail
{
    inline UIPC_GENERIC Eigen::Vector3f xyz(const float4& v)
    {
        return Eigen::Vector3f{v.x, v.y, v.z};
    }

    inline UIPC_GENERIC float max_abs(const float4& v)
    {
        return fmaxf(fabsf(v.x), fmaxf(fabsf(v.y), fabsf(v.z)));
    }

    // max_relative_motion and accd_disp_mag on FP32 displacements.
    template <int NA, int NB>
    inline UIPC_GENERIC void motion_bounds_f32(const Eigen::Vector3f* da,
                                               const Eigen::Vector3f* db,
                                               float& relative_motion,
                                               float& accd_disp)
    {
        Eigen::Vector3f mov = Eigen::Vector3f::Zero();
        float           mag = 0.0f;
        float           m2  = 0.0f;
        for(int i = 0; i < NA; ++i)
        {
            mag = fmaxf(mag, max_abs(da[i]));
            mov += da[i];
        }
        for(int j = 0; j < NB; ++j)
        {
            mag = fmaxf(mag, max_abs(db[j]));
            mov += db[j];
        }
        for(int i = 0; i < NA; ++i)
            for(int j = 0; j < NB; ++j)
                m2 = fmaxf(m2, (da[i] - db[j]).squaredNorm());
        mov *= 1.0f / (NA + NB);
        float ma = 0.0f, mb = 0.0f;
        for(int i = 0; i < NA; ++i)
            ma = fmaxf(ma, (da[i] - mov).squaredNorm());
        for(int j = 0; j < NB; ++j)
            mb = fmaxf(mb, (db[j] - mov).squaredNorm());
        relative_motion = sqrtf(m2) * 1.001f + 1e-5f * mag;
        accd_disp       = (sqrtf(ma) + sqrtf(mb)) * 1.001f + 1e-5f * mag;
    }

    // need of swept_lower_bound.h, from FP32 pair parameters
    inline UIPC_GENERIC float swept_need_f32(float dist, float thickness, float relative_motion, float accd_disp)
    {
        return fmaxf(dist + relative_motion, thickness + static_cast<float>(kAccd) * accd_disp);
    }

    // point_triangle_farther_than's bound on FP32 coordinates relative to the
    // point; false for slivers, as there.
    inline UIPC_GENERIC bool point_triangle_bound_f32(const Eigen::Vector3f& f0,
                                                      const Eigen::Vector3f& f1,
                                                      const Eigen::Vector3f& f2,
                                                      float  scale,
                                                      float& bound)
    {
        if(!(scale > 0.0f))  // every point coincides
            return false;
        const Eigen::Vector3f n  = (f1 - f0).cross(f2 - f0);
        const float           nn = n.squaredNorm();
        const float           s2 = scale * scale;
        if(!(nn >= 1e-2f * s2 * s2))
            return false;
        const float            inv_n = (1.0f / sqrtf(nn));
        const float            h     = fabsf(n.dot(f0)) * inv_n;
        float                  s     = 0.0f;
        const Eigen::Vector3f* v[3]  = {&f0, &f1, &f2};
        for(int e = 0; e < 3; ++e)
        {
            const Eigen::Vector3f& a = *v[e];
            const Eigen::Vector3f& b = *v[(e + 1) % 3];
            const Eigen::Vector3f  m = (b - a).cross(n);
            s = fmaxf(s, -m.dot(a) * (1.0f / sqrtf(m.squaredNorm())));
        }
        bound = sqrtf(h * h + s * s);
        return true;
    }

    // edge_edge_farther_than's bound on FP32 coordinates relative to ea0;
    // false for a degenerate edge, as there.
    inline UIPC_GENERIC bool edge_edge_bound_f32(const Eigen::Vector3f& a1,
                                                 const Eigen::Vector3f& b0,
                                                 const Eigen::Vector3f& b1,
                                                 float                  scale,
                                                 float&                 bound)
    {
        if(!(scale > 0.0f))  // every end point coincides
            return false;
        const Eigen::Vector3f u  = a1;
        const Eigen::Vector3f v  = b1 - b0;
        const float           uu = u.squaredNorm();
        const float           vv = v.squaredNorm();
        const float           s2 = scale * scale;
        if(!(uu >= 1e-6f * s2) || !(vv >= 1e-6f * s2))
            return false;
        const float inv_u = (1.0f / sqrtf(uu));
        const float pu0   = b0.dot(u) * inv_u;
        const float pu1   = b1.dot(u) * inv_u;
        const float g_u =
            interval_gap(0.0f, uu * inv_u, fminf(pu0, pu1), fmaxf(pu0, pu1));
        const float inv_v = (1.0f / sqrtf(vv));
        const float pva   = u.dot(v) * inv_v;
        const float pvb   = b0.dot(v) * inv_v;
        const float g_v =
            interval_gap(fminf(0.0f, pva), fmaxf(0.0f, pva), pvb, pvb + vv * inv_v);
        const Eigen::Vector3f n  = u.cross(v);
        const float           nn = n.squaredNorm();
        float                 h  = 0.0f;
        if(nn >= 1e-2f * uu * vv)
            h = fabsf(n.dot(b0)) * (1.0f / sqrtf(nn));
        bound = sqrtf(h * h + fmaxf(g_u * g_u, g_v * g_v));
        return true;
    }

    inline UIPC_GENERIC bool clearly_farther_f32(float lower_bound, float need, float scale, float rounding)
    {
        return lower_bound > need * 1.001f + 1e-4f * scale + rounding;
    }
}  // namespace detail

// True only if edges ea, eb (FP32 positions with d_hat in w, moving by FP32
// step displacements with the thickness in w) stay farther than d_hat +
// thickness + skin apart for the whole step and ACCD reports no hit; the
// trajectory leaf test's d_hat and thickness rules (EE_d_hat, EE_thickness).
inline UIPC_GENERIC bool edge_edge_swept_farther_than_f32(const float4& ea0,
                                                          const float4& ea1,
                                                          const float4& eb0,
                                                          const float4& eb1,
                                                          const float4& dea0,
                                                          const float4& dea1,
                                                          const float4& deb0,
                                                          const float4& deb1,
                                                          float         skin)
{
    const float           thickness = dea0.w + deb0.w;
    const float           dist      = (ea0.w + eb0.w) * 0.5f + thickness + skin;
    const Eigen::Vector3f da[2]     = {detail::xyz(dea0), detail::xyz(dea1)};
    const Eigen::Vector3f db[2]     = {detail::xyz(deb0), detail::xyz(deb1)};
    float                 motion, accd;
    detail::motion_bounds_f32<2, 2>(da, db, motion, accd);
    const float need = detail::swept_need_f32(dist, thickness, motion, accd);

    const Eigen::Vector3f o = detail::xyz(ea0);
    const Eigen::Vector3f a[2] = {Eigen::Vector3f::Zero(), detail::xyz(ea1) - o};
    const Eigen::Vector3f b[2]  = {detail::xyz(eb0) - o, detail::xyz(eb1) - o};
    const float           scale = fmaxf(detail::max_abs(a[1]),
                              fmaxf(detail::max_abs(b[0]), detail::max_abs(b[1])));
    const float           rounding = 0x1p-20f
                           * fmaxf(fmaxf(detail::max_abs(ea0), detail::max_abs(ea1)),
                                   fmaxf(detail::max_abs(eb0), detail::max_abs(eb1)));
    float bound;
    if(detail::edge_edge_bound_f32(a[1], b[0], b[1], scale, bound)
       && detail::clearly_farther_f32(bound, need, scale, rounding))
        return true;
    const float gap = detail::projected_gap(
        detail::segment_closest_dir(a[0], a[1], b[0], b[1]), a, 2, b, 2);
    return detail::clearly_farther_f32(gap, need, scale, rounding);
}

// True only if point p and triangle t (as above) stay farther than d_hat +
// thickness + skin apart for the whole step and ACCD reports no hit; the
// trajectory leaf test's rules (PT_d_hat, PT_thickness).
inline UIPC_GENERIC bool point_triangle_swept_farther_than_f32(const float4& p,
                                                               const float4& t0,
                                                               const float4& t1,
                                                               const float4& t2,
                                                               const float4& dp,
                                                               const float4& dt0,
                                                               const float4& dt1,
                                                               const float4& dt2,
                                                               float skin)
{
    const float           thickness = dp.w + dt0.w;
    const float           dist      = (p.w + t0.w) * 0.5f + thickness + skin;
    const Eigen::Vector3f da[1]     = {detail::xyz(dp)};
    const Eigen::Vector3f db[3] = {detail::xyz(dt0), detail::xyz(dt1), detail::xyz(dt2)};
    float motion, accd;
    detail::motion_bounds_f32<1, 3>(da, db, motion, accd);
    const float need = detail::swept_need_f32(dist, thickness, motion, accd);

    const Eigen::Vector3f o    = detail::xyz(p);
    const Eigen::Vector3f a[1] = {Eigen::Vector3f::Zero()};
    const Eigen::Vector3f b[3] = {
        detail::xyz(t0) - o, detail::xyz(t1) - o, detail::xyz(t2) - o};
    const float scale    = fmaxf(detail::max_abs(b[0]),
                              fmaxf(detail::max_abs(b[1]), detail::max_abs(b[2])));
    const float rounding = 0x1p-20f
                           * fmaxf(fmaxf(detail::max_abs(p), detail::max_abs(t0)),
                                   fmaxf(detail::max_abs(t1), detail::max_abs(t2)));
    float bound;
    if(detail::point_triangle_bound_f32(b[0], b[1], b[2], scale, bound)
       && detail::clearly_farther_f32(bound, need, scale, rounding))
        return true;
    const float gap = detail::projected_gap(
        detail::triangle_closest_dir(a[0], b[0], b[1], b[2]), a, 1, b, 3);
    return detail::clearly_farther_f32(gap, need, scale, rounding);
}
}  // namespace uipc::backend::cuda::distance
