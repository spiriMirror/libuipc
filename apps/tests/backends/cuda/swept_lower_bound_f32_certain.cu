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

// edge_edge_swept_farther_than_f32_certain must hold only where the first test
// of edge_edge_swept_farther_than_f32 rejects under every FP32 evaluation.
// These cases evaluate that test with explicit roundings (__fmul_rn,
// __fadd_rn, __fmaf_rn, ...): with no multiply fused, with every one fused
// (either product of a sum of two), and with random choices per operation and
// either association of each three-term dot product, next to the function as
// nvcc compiles it. Inputs are random swept edge pairs and pairs placed at the
// test's thresholds: the bound at the threshold, the edge conditioning and the
// normal's conditioning.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/distance/swept_lower_bound.h>
#include <utils/distance/swept_lower_bound_f32.h>
#include <utils/distance/swept_lower_bound_f32_certain.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
// One FP32 evaluation of the source: mode 0 rounds every operation, 1 fuses
// every multiply-add (the first product of a sum of two), 2 likewise with the
// second product and the other association of dot products, 3 picks per
// operation from a seeded generator.
struct Evaluation
{
    int                mode;
    unsigned long long state;

    __device__ int pick(int options)
    {
        if(mode < 3)
            return mode == 0 ? 0 : (mode == 1 ? 1 : options - 1);
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<int>((state >> 33) % static_cast<unsigned long long>(options));
    }
    __device__ float add(float a, float b) { return __fadd_rn(a, b); }
    __device__ float sub(float a, float b) { return __fsub_rn(a, b); }
    __device__ float mul(float a, float b) { return __fmul_rn(a, b); }
    __device__ float root(float a) { return __fsqrt_rn(a); }
    __device__ float inv(float a) { return __fdiv_rn(1.0f, a); }
    __device__ float div(float a, float b) { return __fdiv_rn(a, b); }
    // a * b + c
    __device__ float mul_add(float a, float b, float c)
    {
        return pick(2) ? __fmaf_rn(a, b, c) : __fadd_rn(__fmul_rn(a, b), c);
    }
    // a * b + c * d
    __device__ float sum2(float a, float b, float c, float d)
    {
        switch(pick(3))
        {
            case 0:
                return __fadd_rn(__fmul_rn(a, b), __fmul_rn(c, d));
            case 1:
                return __fmaf_rn(a, b, __fmul_rn(c, d));
            default:
                return __fmaf_rn(c, d, __fmul_rn(a, b));
        }
    }
    __device__ float dot(const Eigen::Vector3f& x, const Eigen::Vector3f& y)
    {
        if(pick(2) == 0)  // (x0 y0 + x1 y1) + x2 y2
            return mul_add(x.z(), y.z(), sum2(x.x(), y.x(), x.y(), y.y()));
        return mul_add(x.x(), y.x(), sum2(x.y(), y.y(), x.z(), y.z()));
    }
    __device__ Eigen::Vector3f sub(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
        return {sub(a.x(), b.x()), sub(a.y(), b.y()), sub(a.z(), b.z())};
    }
    __device__ Eigen::Vector3f cross(const Eigen::Vector3f& u, const Eigen::Vector3f& v)
    {
        return {sum2(u.y(), v.z(), -u.z(), v.y()),
                sum2(u.z(), v.x(), -u.x(), v.z()),
                sum2(u.x(), v.y(), -u.y(), v.x())};
    }
};

__device__ Eigen::Vector3f xyz(const float4& v)
{
    return {v.x, v.y, v.z};
}

__device__ float max_abs(const Eigen::Vector3f& a)
{
    return fmaxf(fabsf(a.x()), fmaxf(fabsf(a.y()), fabsf(a.z())));
}

__device__ float clamp01(float x)
{
    return fminf(fmaxf(x, 0.0f), 1.0f);
}

// Both tests of edge_edge_swept_farther_than_f32 in evaluation e: the first
// (motion_bounds_f32, swept_need_f32, edge_edge_bound_f32 and
// clearly_farther_f32) and the second (segment_closest_dir, projected_gap and
// clearly_farther_f32), each evaluated whatever the other gives.
__device__ void evaluate_tests(
    Evaluation& e, const float4* x, const float4* d, float skin, bool& first, bool& second)
{
    const float thickness = e.add(d[0].w, d[2].w);
    const float dist = e.add(e.mul_add(e.add(x[0].w, x[2].w), 0.5f, thickness), skin);

    const Eigen::Vector3f da[2] = {xyz(d[0]), xyz(d[1])};
    const Eigen::Vector3f db[2] = {xyz(d[2]), xyz(d[3])};
    float                 mag   = 0.0f;
    Eigen::Vector3f       mov   = Eigen::Vector3f::Zero();
    for(int i = 0; i < 2; ++i)
    {
        mag = fmaxf(mag, max_abs(da[i]));
        mov = {e.add(mov.x(), da[i].x()),
               e.add(mov.y(), da[i].y()),
               e.add(mov.z(), da[i].z())};
    }
    for(int j = 0; j < 2; ++j)
    {
        mag = fmaxf(mag, max_abs(db[j]));
        mov = {e.add(mov.x(), db[j].x()),
               e.add(mov.y(), db[j].y()),
               e.add(mov.z(), db[j].z())};
    }
    float m2 = 0.0f;
    for(int i = 0; i < 2; ++i)
        for(int j = 0; j < 2; ++j)
        {
            const Eigen::Vector3f r = e.sub(da[i], db[j]);
            m2                      = fmaxf(m2, e.dot(r, r));
        }
    mov = {e.mul(mov.x(), 0.25f), e.mul(mov.y(), 0.25f), e.mul(mov.z(), 0.25f)};
    float ma = 0.0f, mb = 0.0f;
    for(int i = 0; i < 2; ++i)
    {
        const Eigen::Vector3f r = e.sub(da[i], mov);
        ma                      = fmaxf(ma, e.dot(r, r));
    }
    for(int j = 0; j < 2; ++j)
    {
        const Eigen::Vector3f r = e.sub(db[j], mov);
        mb                      = fmaxf(mb, e.dot(r, r));
    }
    const float motion = e.sum2(e.root(m2), 1.001f, 1e-5f, mag);
    const float accd = e.sum2(e.add(e.root(ma), e.root(mb)), 1.001f, 1e-5f, mag);
    const float need =
        fmaxf(e.add(dist, motion),
              e.mul_add(static_cast<float>(distance::detail::kAccd), accd, thickness));

    const Eigen::Vector3f o  = xyz(x[0]);
    const Eigen::Vector3f a1 = e.sub(xyz(x[1]), o);
    const Eigen::Vector3f b0 = e.sub(xyz(x[2]), o);
    const Eigen::Vector3f b1 = e.sub(xyz(x[3]), o);
    const float scale = fmaxf(max_abs(a1), fmaxf(max_abs(b0), max_abs(b1)));
    float       big   = 0.0f;
    for(int k = 0; k < 4; ++k)
        big = fmaxf(big, max_abs(xyz(x[k])));
    // need * 1.001 + 1e-4 scale + rounding, rounding = 2^-20 big (exact),
    // evaluated for each test
    const float T1 = e.mul_add(0x1p-20f, big, e.sum2(need, 1.001f, 1e-4f, scale));
    const float T2 = e.mul_add(0x1p-20f, big, e.sum2(need, 1.001f, 1e-4f, scale));

    // first test
    first = false;
    {
        const Eigen::Vector3f u   = a1;
        const Eigen::Vector3f v   = e.sub(b1, b0);
        const float           uu  = e.dot(u, u);
        const float           vv  = e.dot(v, v);
        const float           thr = e.mul(1e-6f, e.mul(scale, scale));
        if(uu >= thr && vv >= thr)
        {
            const float inv_u = e.inv(e.root(uu));
            const float pu0   = e.mul(e.dot(b0, u), inv_u);
            const float pu1   = e.mul(e.dot(b1, u), inv_u);
            // interval_gap(0, uu inv_u, min, max): the product may fuse into the subtraction
            const float g_u   = fmaxf(0.0f,
                                    fmaxf(e.mul_add(-uu, inv_u, fminf(pu0, pu1)),
                                          e.sub(0.0f, fmaxf(pu0, pu1))));
            const float inv_v = e.inv(e.root(vv));
            const float pva   = e.mul(e.dot(u, v), inv_v);
            const float dvb   = e.dot(b0, v);
            // pvb = dvb inv_v may fuse into either use: pvb - max(0, pva), pvb + vv inv_v
            const float lo  = e.mul_add(dvb, inv_v, -fmaxf(0.0f, pva));
            const float top = e.sum2(dvb, inv_v, vv, inv_v);
            const float g_v = fmaxf(0.0f, fmaxf(lo, e.sub(fminf(0.0f, pva), top)));
            const Eigen::Vector3f n  = e.cross(u, v);
            const float           nn = e.dot(n, n);
            float                 h  = 0.0f;
            if(nn >= e.mul(e.mul(1e-2f, uu), vv))
                h = e.mul(fabsf(e.dot(n, b0)), e.inv(e.root(nn)));
            const float bound =
                e.root(e.mul_add(h, h, fmaxf(e.mul(g_u, g_u), e.mul(g_v, g_v))));
            first = bound > T1;
        }
    }

    // second test: segment_closest_dir(0, a1, b0, b1), then projected_gap
    {
        const Eigen::Vector3f d1  = a1;             // a1 - 0
        const Eigen::Vector3f d2  = e.sub(b1, b0);  // b1 - b0
        const Eigen::Vector3f r   = -b0;            // 0 - b0
        const float           a   = e.dot(d1, d1);
        const float           ee  = e.dot(d2, d2);
        const float           f   = e.dot(d2, r);
        const float           c   = e.dot(d1, r);
        const float           b   = e.dot(d1, d2);
        const float           den = e.sum2(a, ee, -b, b);
        float s = (den > e.mul(e.mul(1e-12f, a), ee) && den > 0.0f) ?
                      clamp01(e.div(e.sum2(b, f, -c, ee), den)) :
                      0.0f;
        float t = ee > 0.0f ? e.div(e.mul_add(b, s, f), ee) : 0.0f;
        if(t < 0.0f)
        {
            t = 0.0f;
            s = a > 0.0f ? clamp01(e.div(-c, a)) : 0.0f;
        }
        else if(t > 1.0f)
        {
            t = 1.0f;
            s = a > 0.0f ? clamp01(e.div(e.sub(b, c), a)) : 0.0f;
        }
        Eigen::Vector3f w;
        for(int k = 0; k < 3; ++k)
            w[k] = e.sub(e.mul_add(d1[k], s, 0.0f), e.mul_add(d2[k], t, b0[k]));
        float       gap = 0.0f;
        const float ww  = e.dot(w, w);
        if(ww > 0.0f)
        {
            const float q = e.inv(e.root(ww));
            const Eigen::Vector3f uv{e.mul(w.x(), q), e.mul(w.y(), q), e.mul(w.z(), q)};
            float       a_lo = e.dot(uv, Eigen::Vector3f::Zero()), a_hi = a_lo;
            const float ta   = e.dot(uv, a1);
            a_lo             = fminf(a_lo, ta);
            a_hi             = fmaxf(a_hi, ta);
            float       b_lo = e.dot(uv, b0), b_hi = b_lo;
            const float tb = e.dot(uv, b1);
            b_lo           = fminf(b_lo, tb);
            b_hi           = fmaxf(b_hi, tb);
            gap = e.mul(fmaxf(0.0f, fmaxf(e.sub(b_lo, a_hi), e.sub(a_lo, b_hi))),
                        1.0f - 1e-5f);
        }
        second = gap > T2;
    }
}

constexpr int Variants = 16;  // modes 0, 1, 2, then 13 seeded random ones
// outcome bits: the first test per variant, the second per variant, then
constexpr int CertainBoundBit = 2 * Variants;
constexpr int CertainGapBit   = 2 * Variants + 1;
constexpr int CompiledBit     = 2 * Variants + 2;
constexpr int ConditionedBit = 2 * Variants + 3;  // the certain tests' ranges and conditioning hold

__global__ void swept_lower_bound_f32_certain_kernel(const float4* xs,
                                                     const float4* ds,
                                                     const float*  skins,
                                                     int           n,
                                                     unsigned long long* outcomes)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    const float4*      x       = xs + 4 * i;
    const float4*      d       = ds + 4 * i;
    const float        skin    = skins[i];
    unsigned long long outcome = 0;
    for(int k = 0; k < Variants; ++k)
    {
        Evaluation e{k < 3 ? k : 3,
                     0x9e3779b97f4a7c15ull * static_cast<unsigned long long>(i + 1)
                         + static_cast<unsigned long long>(k) * 0xbf58476d1ce4e5b9ull};
        bool first = false, second = false;
        evaluate_tests(e, x, d, skin, first, second);
        outcome |= static_cast<unsigned long long>(first) << k;
        outcome |= static_cast<unsigned long long>(second) << (Variants + k);
    }
    if(distance::edge_edge_swept_farther_than_f32_certain_bound(
           x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], skin))
        outcome |= 1ull << CertainBoundBit;
    if(distance::edge_edge_swept_farther_than_f32_certain_gap(
           x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], skin))
        outcome |= 1ull << CertainGapBit;
    if(distance::edge_edge_swept_farther_than_f32(
           x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], skin))
        outcome |= 1ull << CompiledBit;
    distance::detail::CertainEdgeEdge c;
    if(distance::detail::certain_edge_edge(
           x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], skin, c))
        outcome |= 1ull << ConditionedBit;
    outcomes[i] = outcome;
}

class Sampler
{
  public:
    explicit Sampler(std::uint64_t seed)
        : m_rng{seed}
    {
    }
    double uniform(double lo, double hi)
    {
        return std::uniform_real_distribution<double>{lo, hi}(m_rng);
    }
    double log_uniform(double lo, double hi)
    {
        return std::pow(10.0, uniform(lo, hi));
    }
    double sign() { return uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0; }
    int    pick(int n)
    {
        return std::uniform_int_distribution<int>{0, n - 1}(m_rng);
    }
    Eigen::Vector3d box(double half)
    {
        return Eigen::Vector3d{uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0)} * half;
    }
    Eigen::Vector3d direction()
    {
        while(true)
        {
            const Eigen::Vector3d v  = box(1.0);
            const double          n2 = v.squaredNorm();
            if(n2 > 1e-4 && n2 <= 1.0)
                return v / std::sqrt(n2);
        }
    }

  private:
    std::mt19937_64 m_rng;
};

// A case in FP32 inputs: positions (x, y, z, d_hat), steps (dx, dy, dz,
// thickness), skin.
struct Case
{
    std::array<float4, 4> x;
    std::array<float4, 4> d;
    float                 skin;
    int                   kind;
};

constexpr int KindRandom    = 0;  // random swept pairs, d_hat drawn
constexpr int KindThreshold = 1;  // bound at the threshold
constexpr int KindEdge      = 2;  // an edge at its conditioning threshold
constexpr int KindNormal    = 3;  // the normal at its conditioning threshold
constexpr int KindGap       = 4;  // the second test's gap at the threshold
constexpr int Kinds         = 5;

Eigen::Vector3d xyz_d(const float4& v)
{
    return {v.x, v.y, v.z};
}

// edge_edge_bound_f32 and the motion terms in FP64 on the FP32 relative
// coordinates: the real values that the FP32 evaluations approximate.
struct Reference
{
    double bound = 0.0;  // < 0: a degenerate edge
    double gap   = 0.0;  // projected_gap along segment_closest_dir
    double motion = 0.0, kaccd = 0.0, rest = 0.0;  // rest: 1e-4 scale + rounding
};

// segment_closest_dir and projected_gap of swept_lower_bound.h in FP64
double reference_gap(const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1)
{
    const Eigen::Vector3d d1 = a1, d2 = b1 - b0, r = -b0;
    const double a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
    const double c = d1.dot(r), b = d1.dot(d2), den = a * e - b * b;
    auto clamp = [](double x) { return std::min(std::max(x, 0.0), 1.0); };
    double s = den > 1e-12 * a * e && den > 0.0 ? clamp((b * f - c * e) / den) : 0.0;
    double t = e > 0.0 ? (b * s + f) / e : 0.0;
    if(t < 0.0)
    {
        t = 0.0;
        s = a > 0.0 ? clamp(-c / a) : 0.0;
    }
    else if(t > 1.0)
    {
        t = 1.0;
        s = a > 0.0 ? clamp((b - c) / a) : 0.0;
    }
    const Eigen::Vector3d w  = d1 * s - (b0 + d2 * t);
    const double          ww = w.squaredNorm();
    if(!(ww > 0.0))
        return 0.0;
    const Eigen::Vector3d u  = w / std::sqrt(ww);
    const double          pa = u.dot(a1), p0 = u.dot(b0), p1 = u.dot(b1);
    return std::max(0.0,
                    std::max(std::min(p0, p1) - std::max(0.0, pa),
                             std::min(0.0, pa) - std::max(p0, p1)));
}

Reference reference(const Case& c)
{
    Reference       r;
    const float4*   x     = c.x.data();
    const float4*   d     = c.d.data();
    Eigen::Vector3d da[2] = {xyz_d(d[0]), xyz_d(d[1])};
    Eigen::Vector3d db[2] = {xyz_d(d[2]), xyz_d(d[3])};
    double          mag   = 0.0;
    Eigen::Vector3d mov   = Eigen::Vector3d::Zero();
    double          m2 = 0.0, ma = 0.0, mb = 0.0;
    for(auto& v : da)
        mag = std::max(mag, v.cwiseAbs().maxCoeff()), mov += v;
    for(auto& v : db)
        mag = std::max(mag, v.cwiseAbs().maxCoeff()), mov += v;
    mov *= 0.25;
    for(auto& a : da)
        for(auto& b : db)
            m2 = std::max(m2, (a - b).squaredNorm());
    for(auto& a : da)
        ma = std::max(ma, (a - mov).squaredNorm());
    for(auto& b : db)
        mb = std::max(mb, (b - mov).squaredNorm());
    r.motion = std::sqrt(m2) * 1.001 + 1e-5 * mag;
    r.kaccd  = distance::detail::kAccd
              * ((std::sqrt(ma) + std::sqrt(mb)) * 1.001 + 1e-5 * mag);

    auto rel = [&](int k)
    {
        return Eigen::Vector3d{static_cast<double>(x[k].x - x[0].x),
                               static_cast<double>(x[k].y - x[0].y),
                               static_cast<double>(x[k].z - x[0].z)};
    };
    const Eigen::Vector3d a1 = rel(1), b0 = rel(2), b1 = rel(3);
    const double          scale =
        std::max(a1.cwiseAbs().maxCoeff(),
                 std::max(b0.cwiseAbs().maxCoeff(), b1.cwiseAbs().maxCoeff()));
    double big = 0.0;
    for(int k = 0; k < 4; ++k)
        big = std::max(big, xyz_d(x[k]).cwiseAbs().maxCoeff());
    r.rest = 1e-4 * scale + std::ldexp(big, -20);
    r.gap  = reference_gap(a1, b0, b1);

    const Eigen::Vector3d u  = a1;
    const Eigen::Vector3d v  = b1 - b0;
    const double          uu = u.squaredNorm(), vv = v.squaredNorm();
    if(!(uu >= 1e-6 * scale * scale) || !(vv >= 1e-6 * scale * scale))
    {
        r.bound = -1.0;
        return r;
    }
    const double pu0 = b0.dot(u) / std::sqrt(uu), pu1 = b1.dot(u) / std::sqrt(uu);
    const double g_u =
        std::max(0.0, std::max(std::min(pu0, pu1) - std::sqrt(uu), -std::max(pu0, pu1)));
    const double pva = u.dot(v) / std::sqrt(vv), pvb = b0.dot(v) / std::sqrt(vv);
    const double          g_v = std::max(0.0,
                                std::max(pvb - std::max(0.0, pva),
                                         std::min(0.0, pva) - (pvb + std::sqrt(vv))));
    const Eigen::Vector3d nrm = u.cross(v);
    double                h   = 0.0;
    if(nrm.squaredNorm() >= 1e-2 * uu * vv)
        h = std::abs(nrm.dot(b0)) / nrm.norm();
    r.bound = std::sqrt(h * h + std::max(g_u * g_u, g_v * g_v));
    return r;
}

float4 to_float4(const Eigen::Vector3d& v, double w)
{
    return make_float4(static_cast<float>(v.x()),
                       static_cast<float>(v.y()),
                       static_cast<float>(v.z()),
                       static_cast<float>(w));
}

// Puts the threshold at `target` (thickness and d_hat), if the motion leaves room.
bool place_threshold(Case& c, double target, Sampler& s)
{
    const Reference r    = reference(c);
    const double    need = (target - r.rest) / 1.001;
    const double    room = need - std::max(r.motion, r.kaccd);
    if(!(room > 0.0))
        return false;
    const double thickness = s.uniform(0.0, 1.0) < 0.3 ? 0.0 : s.uniform(0.0, 1.0) * room;
    const double d_hat = need - thickness - r.motion;  // skin 0
    c.d[0].w = c.d[2].w = static_cast<float>(0.5 * thickness);
    c.x[0].w = c.x[2].w = static_cast<float>(d_hat);
    c.skin              = 0.0f;
    return true;
}

Eigen::Vector3d rounded(const Eigen::Vector3d& v)
{
    const float4 f = to_float4(v, 0.0);
    return {f.x, f.y, f.z};
}

Case make_case(Sampler& s, int kind)
{
    Case c;
    c.kind = kind;
    // Random pairs lie up to 1e3 from the origin; at a conditioning threshold
    // the first vertex is the origin, so the FP32 positions keep the placement.
    const bool random_pair = kind == KindRandom || kind == KindThreshold || kind == KindGap;
    const Eigen::Vector3d o = random_pair && s.uniform(0.0, 1.0) < 0.5 ? s.box(1e3) :
                              random_pair ? s.box(1.0) :
                                            Eigen::Vector3d::Zero();
    const double size = s.log_uniform(-4.0, 0.0);
    Eigen::Vector3d a0 = o + (random_pair ? s.box(size) : Eigen::Vector3d::Zero());
    Eigen::Vector3d       a1 = a0 + s.box(size);
    Eigen::Vector3d       b0 = a0 + s.box(3.0 * size);
    Eigen::Vector3d       b1 = b0 + s.box(size);
    const Eigen::Vector3d ea = a1 - a0;
    switch(random_pair ? s.pick(8) : 0)
    {
        case 1:  // nearly parallel
            b1 = b0 + ea * s.uniform(0.2, 1.5) + s.box(size * s.log_uniform(-7.0, -2.0));
            break;
        case 2:  // parallel up to rounding
            b1 = b0 + ea * s.uniform(0.2, 1.5);
            break;
        case 3:  // crossing at a small distance
        {
            const Eigen::Vector3d mid = a0 + ea * s.uniform(0.1, 0.9);
            const Eigen::Vector3d dir = s.box(size);
            const Eigen::Vector3d off = ea.cross(dir).normalized()
                                        * (s.sign() * size * s.log_uniform(-6.0, 0.0));
            const double t = s.uniform(0.1, 0.9);
            b0             = mid + off - dir * t;
            b1             = mid + off + dir * (1.0 - t);
            break;
        }
        case 4:  // tiny edge
            b1 = b0 + s.box(size * s.log_uniform(-5.0, -2.0));
            break;
        case 5:  // collinear, apart along the line
            b0 = a1 + ea * s.log_uniform(-4.0, 0.0);
            b1 = b0 + ea * s.uniform(0.1, 1.0);
            break;
        case 6:  // endpoint near endpoint
            b0 = a1 + s.box(size * s.log_uniform(-6.0, -1.0));
            break;
        case 7:  // far apart
            b0 = a0 + s.direction() * size * s.log_uniform(0.5, 2.0);
            b1 = b0 + s.box(size);
            break;
        default:
            break;
    }

    // a relative change of 1e-8 .. 1e-4 either way
    auto nudge = [&] { return 1.0 + s.sign() * s.log_uniform(-8.0, -4.0); };
    if(kind == KindGap)
    {
        // where the second test decides: nearly parallel edges side by side,
        // closest points at an end, or the random shapes above
        const Eigen::Vector3d dir  = s.direction();
        Eigen::Vector3d       side = s.direction();
        side                       = (side - dir * dir.dot(side)).normalized();
        const double len           = size * s.uniform(0.5, 1.0);
        const double away          = size * s.log_uniform(-3.0, 0.0);
        switch(s.pick(3))
        {
            case 0:  // nearly parallel, overlapping or shifted along the line
            {
                const double angle =
                    s.uniform(0.0, 1.0) < 0.3 ? 0.0 : s.log_uniform(-7.0, -1.0);
                const Eigen::Vector3d dir2 =
                    (dir * std::cos(angle) + side.cross(dir) * std::sin(angle)).normalized();
                a1 = a0 + dir * len;
                b0 = a0 + side * away + dir * (len * s.uniform(-1.2, 1.2));
                b1 = b0 + dir2 * (len * s.uniform(0.3, 1.5) * s.sign());
                break;
            }
            case 1:  // the second edge beyond the first's end
                a1 = a0 + dir * len;
                b0 = a1 + (dir * s.uniform(0.0, 1.0) + side).normalized() * away;
                b1 = b0 + s.direction() * (len * s.uniform(0.3, 1.5));
                break;
            default:
                break;
        }
    }
    if(kind == KindEdge)
    {
        // |u|^2 or |v|^2 at 1e-6 scale^2 (a0 = 0: the coordinates are relative)
        if(s.pick(2) == 0)
        {
            b0 = rounded(b0);
            b1 = rounded(b1);
            const double scale =
                std::max(b0.cwiseAbs().maxCoeff(), b1.cwiseAbs().maxCoeff());
            a1 = s.direction() * (1e-3 * scale * std::sqrt(nudge()));
        }
        else
        {
            // v of the rounded positions; the scale set by a1's largest coordinate
            const double len = size * s.log_uniform(-3.0, -1.0);
            b0               = rounded(s.box(300.0 * len));
            b1               = rounded(b0 + s.direction() * len);
            const Eigen::Vector3d v{static_cast<double>(static_cast<float>(b1.x())
                                                        - static_cast<float>(b0.x())),
                                    static_cast<double>(static_cast<float>(b1.y())
                                                        - static_cast<float>(b0.y())),
                                    static_cast<double>(static_cast<float>(b1.z())
                                                        - static_cast<float>(b0.z()))};
            const double scale = 1e3 * v.norm() * std::sqrt(nudge());
            a1                 = Eigen::Vector3d{s.sign() * scale,
                                 s.uniform(-0.5, 0.5) * scale,
                                 s.uniform(-0.5, 0.5) * scale};
        }
    }
    else if(kind == KindNormal)
    {
        // |u x v|^2 at 1e-2 |u|^2 |v|^2: sin^2 of the angle at 1e-2
        const Eigen::Vector3d ud = ea.normalized();
        Eigen::Vector3d       w  = s.direction();
        w                        = (w - ud * ud.dot(w)).normalized();
        const double sin2        = 1e-2 * nudge();
        const Eigen::Vector3d vd = ud * std::sqrt(1.0 - sin2) + w * std::sqrt(sin2);
        b1 = b0 + vd * ea.norm() * s.uniform(0.2, 2.0);
    }

    // step displacements: a common translation, the second edge's motion and
    // jitter; smaller at the thresholds, which must stay above the motion
    const double    rel = kind == KindRandom ? s.log_uniform(-5.0, 0.0) :
                                               s.log_uniform(-7.0, -2.0);
    Eigen::Vector3d T   = s.uniform(0.0, 1.0) < 0.5 ?
                              s.box(size * s.log_uniform(-2.0, 1.0)) :
                              Eigen::Vector3d::Zero();
    Eigen::Vector3d W   = s.box(size * rel);
    std::array<Eigen::Vector3d, 4> step;
    for(int k = 0; k < 4; ++k)
    {
        step[k] = T + (k >= 2 ? W : Eigen::Vector3d::Zero());
        if(s.uniform(0.0, 1.0) < 0.3)
            step[k] += s.box(size * rel * s.log_uniform(-3.0, 0.0));
    }

    const Eigen::Vector3d p[4]  = {a0, a1, b0, b1};
    const double          d_hat = size * s.log_uniform(-4.0, 0.0);
    const double          thickness =
        s.uniform(0.0, 1.0) < 0.5 ? 0.0 : size * s.log_uniform(-4.0, -1.0);
    for(int k = 0; k < 4; ++k)
    {
        c.x[k] = to_float4(p[k], d_hat);
        c.d[k] = to_float4(step[k], 0.5 * thickness);
    }
    c.skin = s.uniform(0.0, 1.0) < 0.5 ?
                 0.0f :
                 static_cast<float>(size * s.log_uniform(-4.0, -1.0));

    const Reference r = reference(c);
    if(kind == KindGap)
    {
        if(r.gap > 0.0)
            place_threshold(c, r.gap * nudge(), s);
    }
    else if(r.bound > 0.0 && kind == KindThreshold)
        place_threshold(c, r.bound * nudge(), s);
    else if(r.bound > 0.0 && kind != KindRandom)
        // below the bound, so that the conditioning decides
        place_threshold(c, r.bound * s.uniform(0.3, 0.999), s);
    return c;
}

std::string describe(const Case& c)
{
    std::ostringstream os;
    os.precision(9);
    for(int k = 0; k < 4; ++k)
        os << " x" << k << " (" << c.x[k].x << ", " << c.x[k].y << ", "
           << c.x[k].z << ", " << c.x[k].w << ")";
    for(int k = 0; k < 4; ++k)
        os << " d" << k << " (" << c.d[k].x << ", " << c.d[k].y << ", "
           << c.d[k].z << ", " << c.d[k].w << ")";
    os << " skin " << c.skin << " kind " << c.kind;
    return os.str();
}

struct Stats
{
    SizeT cases         = 0;
    SizeT certain_bound = 0, certain_gap = 0, certain = 0;
    SizeT all_first = 0, all_second = 0, all_either = 0;  // every evaluation rejects
    SizeT conditioned_either = 0, conditioned_certain = 0;  // of those conditioned
    SizeT split_first = 0, split_second = 0;  // the evaluations disagree
    // a certain test holds but some evaluation keeps the pair in that test,
    // or the compiled function keeps it
    SizeT wrong_bound = 0, wrong_gap = 0, wrong_compiled = 0;
    int   first_wrong = -1;
};

void check_certain(std::uint64_t seed)
{
    constexpr int     n = 1 << 18;
    Sampler           s{seed};
    std::vector<Case> cases;
    cases.reserve(n);
    std::vector<float4> xs, ds;
    std::vector<float>  skins;
    for(int i = 0; i < n; ++i)
    {
        cases.push_back(make_case(s, i % Kinds));
        const Case& c = cases.back();
        xs.insert(xs.end(), c.x.begin(), c.x.end());
        ds.insert(ds.end(), c.d.begin(), c.d.end());
        skins.push_back(c.skin);
    }

    cuda_tool::DeviceBuffer<float4>             x_buf, d_buf;
    cuda_tool::DeviceBuffer<float>              skin_buf;
    cuda_tool::DeviceBuffer<unsigned long long> outcome_buf;
    x_buf.copy_from(xs.data(), xs.size());
    d_buf.copy_from(ds.data(), ds.size());
    skin_buf.copy_from(skins.data(), skins.size());
    outcome_buf.resize_discard(n);
    constexpr int block = 128;
    swept_lower_bound_f32_certain_kernel<<<(n + block - 1) / block, block>>>(
        x_buf.data(), d_buf.data(), skin_buf.data(), n, outcome_buf.data());
    CUDA_TOOL_CHECK(cudaGetLastError());
    std::vector<unsigned long long> outcomes;
    outcome_buf.copy_to(outcomes);

    std::array<Stats, Kinds>     stats;
    constexpr unsigned long long all = (1ull << Variants) - 1;
    for(int i = 0; i < n; ++i)
    {
        Stats&                   st     = stats[cases[i].kind];
        const unsigned long long o      = outcomes[i];
        const unsigned long long first  = o & all;
        const unsigned long long second = (o >> Variants) & all;
        const bool               cb     = (o >> CertainBoundBit) & 1;
        const bool               cg     = (o >> CertainGapBit) & 1;
        const bool               kept   = !((o >> CompiledBit) & 1);
        ++st.cases;
        st.certain_bound += cb;
        st.certain_gap += cg;
        st.certain += cb || cg;
        st.all_first += first == all;
        st.all_second += second == all;
        st.all_either += (first | second) == all;
        if((o >> ConditionedBit) & 1)
        {
            st.conditioned_either += (first | second) == all;
            st.conditioned_certain += cb || cg;
        }
        st.split_first += first != 0 && first != all;
        st.split_second += second != 0 && second != all;
        const bool wb = cb && first != all;
        const bool wg = cg && second != all;
        const bool wc = (cb || cg) && kept;
        st.wrong_bound += wb;
        st.wrong_gap += wg;
        st.wrong_compiled += wc;
        if((wb || wg || wc) && st.first_wrong < 0)
            st.first_wrong = i;
    }
    for(int k = 0; k < Kinds; ++k)
    {
        const Stats& st = stats[k];
        CAPTURE(k,
                st.cases,
                st.certain_bound,
                st.certain_gap,
                st.certain,
                st.all_first,
                st.all_second,
                st.all_either,
                st.split_first,
                st.split_second);
        CAPTURE(st.conditioned_either, st.conditioned_certain);
        CAPTURE(st.wrong_bound, st.wrong_gap, st.wrong_compiled);
        INFO("first wrong: "
             << (st.first_wrong >= 0 ? describe(cases[st.first_wrong]) : "none"));
        REQUIRE(st.wrong_bound + st.wrong_gap + st.wrong_compiled == 0);
    }
    // the evaluations must disagree at the thresholds (the cases reach them)
    REQUIRE(stats[KindThreshold].split_first > 0);
    REQUIRE(stats[KindEdge].split_first + stats[KindNormal].split_first > 0);
    REQUIRE(stats[KindGap].split_second > 0);
    // and the certain tests must decide most random pairs that every
    // evaluation rejects (both need conditioned edges; a tiny edge is not)
    REQUIRE(stats[KindRandom].certain_bound * 10 >= stats[KindRandom].all_first * 9);
    REQUIRE(stats[KindRandom].conditioned_certain * 100
            >= stats[KindRandom].conditioned_either * 99);
}
}  // namespace

TEST_CASE("swept_lower_bound_f32_certain", "[cuda][collision detection][swept_lower_bound]")
{
    check_certain(23);
    check_certain(29);
}
