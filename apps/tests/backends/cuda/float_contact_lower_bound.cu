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

// The broad-phase leaf predicates of info_stackless_bvh reject candidates with
// FP32 lower bounds before their FP64 tests. These cases evaluate the bounds
// and their FP64 references in device code, as the predicates do, on random
// and adversarial near-contact configurations placed far from the origin.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance/float_lower_bound.h>
#include <utils/distance.h>  // swept box test and ACCD (ccd.h)
#include <utils/distance/swept_lower_bound.h>
#include <utils/distance/swept_lower_bound_f32.h>
#include <utils/step_displacement.h>

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
// PT: p, t0, t1, t2. EE: a0, a1, b0, b1.
using Simplex4 = std::array<Vector3, 4>;

struct Sample
{
    Simplex4 x;
    Float    size;  // characteristic primitive size
    int      mode;  // generator branch, 0 = generic
};

class Sampler
{
  public:
    explicit Sampler(std::uint64_t seed)
        : m_rng{seed}
    {
    }

    Float uniform(Float lo, Float hi)
    {
        return std::uniform_real_distribution<Float>{lo, hi}(m_rng);
    }
    Float log_uniform(Float lo, Float hi)
    {
        return std::pow(10.0, uniform(lo, hi));
    }
    Float sign() { return uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0; }
    int   pick(int n)
    {
        return std::uniform_int_distribution<int>{0, n - 1}(m_rng);
    }
    Vector3 box(Float half)
    {
        return Vector3{uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0)} * half;
    }
    Vector3 direction()
    {
        while(true)
        {
            Vector3 v  = box(1.0);
            Float   n2 = v.squaredNorm();
            if(n2 > 1e-4 && n2 <= 1.0)
                return v / std::sqrt(n2);
        }
    }

  private:
    std::mt19937_64 m_rng;
};

Vector3 unit_or_z(const Vector3& v)
{
    Float l = v.norm();
    return l > 0.0 ? Vector3(v / l) : Vector3(0.0, 0.0, 1.0);
}

// Primitives of size 1e-4 .. 1 around an offset of up to 1e3, so that the
// FP64-relative coordinates matter, from benign to adversarial shapes.
Sample sample_point_triangle(Sampler& s)
{
    const Vector3 o    = s.box(1e3);
    const Float   size = s.log_uniform(-4.0, 0.0);
    Vector3       t0   = o + s.box(size);
    Vector3       t1   = o + s.box(size);
    Vector3       t2   = o + s.box(size);
    Vector3       p    = o + s.box(3.0 * size);
    const Vector3 n    = unit_or_z((t1 - t0).cross(t2 - t0));
    const Float   a    = s.uniform(0.0, 1.0);
    const Float   b    = s.uniform(0.0, 1.0 - a);
    const Vector3 c    = t0 + a * (t1 - t0) + b * (t2 - t0);  // interior point
    const int     mode = s.pick(9);
    switch(mode)
    {
        case 1:  // sliver: the third vertex almost on the first edge
            t2 = t0 + (t1 - t0) * s.uniform(-1.0, 1.0) + s.box(size * 1e-6);
            break;
        case 2:  // close to the plane above the interior (contact band)
            p = c + n * (s.sign() * size * s.log_uniform(-9.0, -1.0));
            break;
        case 3:  // near an edge-region boundary
            p = t0 + (t1 - t0) * 0.5 + s.box(size * 1e-3);
            break;
        case 4:  // near a vertex region
            p = t0 + s.box(size * 1e-3);
            break;
        case 5:  // in the plane, just outside the first edge
        {
            Vector3 out = unit_or_z((t1 - t0).cross(n));
            if(out.dot(t2 - t0) > 0.0)
                out = -out;
            p = t0 + (t1 - t0) * s.uniform(0.0, 1.0)
                + out * (size * s.log_uniform(-8.0, -1.0))
                + n * (s.sign() * size * s.log_uniform(-9.0, -3.0));
            break;
        }
        case 6:  // tiny triangle a few sizes away from the point
            t1 = t0 + s.box(size * 1e-4);
            t2 = t0 + s.box(size * 1e-4);
            break;
        case 7:  // collinear triangle
            t2 = t0 + (t1 - t0) * 0.5;
            break;
        case 8:  // in the plane, just beyond a vertex
            p = t0 + unit_or_z(t0 - c) * (size * s.log_uniform(-8.0, -1.0));
            break;
        default:  // generic
            break;
    }
    return {{p, t0, t1, t2}, size, mode};
}

Sample sample_edge_edge(Sampler& s)
{
    const Vector3 o    = s.box(1e3);
    const Float   size = s.log_uniform(-4.0, 0.0);
    Vector3       a0   = o + s.box(size);
    Vector3       a1   = o + s.box(size);
    Vector3       b0   = o + s.box(size);
    Vector3       b1   = o + s.box(size);
    const Vector3 ea   = a1 - a0;
    const int     mode = s.pick(8);
    switch(mode)
    {
        case 1:  // nearly parallel
            b1 = b0 + ea * s.uniform(0.2, 1.5) + s.box(size * 1e-5);
            break;
        case 2:  // parallel up to rounding
            b1 = b0 + ea * s.uniform(0.2, 1.5);
            break;
        case 3:  // nearly intersecting, closest points interior
        {
            const Vector3 mid = a0 + ea * s.uniform(0.1, 0.9);
            const Vector3 dir = s.box(size);
            const Vector3 off = unit_or_z(ea.cross(dir))
                                * (s.sign() * size * s.log_uniform(-9.0, -1.0));
            const Float u = s.uniform(0.1, 0.9);
            b0            = mid + off - dir * u;
            b1            = mid + off + dir * (1.0 - u);
            break;
        }
        case 4:  // tiny edge
            b1 = b0 + s.box(size * 1e-4);
            break;
        case 5:  // collinear, separated along the line
            b0 = a1 + ea * s.log_uniform(-6.0, 0.0) + s.box(size * 1e-9);
            b1 = b0 + ea * s.uniform(0.1, 1.0);
            break;
        case 6:  // endpoint close to endpoint
            b0 = a1 + s.box(size * s.log_uniform(-8.0, -1.0));
            break;
        case 7:  // second edge a few sizes away
            b0 = o + s.box(3.0 * size);
            b1 = b0 + s.box(size);
            break;
        default:  // generic
            break;
    }
    return {{a0, a1, b0, b1}, size, mode};
}

std::string describe(const char* name, const Simplex4& x)
{
    std::ostringstream os;
    os.precision(17);
    os << name << ":";
    for(const Vector3& v : x)
        os << " (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return os.str();
}

UIPC_DEVICE Float exact_distance2(bool           point_triangle,
                                  const Vector3& x0,
                                  const Vector3& x1,
                                  const Vector3& x2,
                                  const Vector3& x3)
{
    Float D2 = -1.0;
    if(point_triangle)
    {
        Vector4i flag = distance::point_triangle_distance_flag(x0, x1, x2, x3);
        distance::point_triangle_distance2(flag, x0, x1, x2, x3, D2);
    }
    else
    {
        Vector4i flag = distance::edge_edge_distance_flag(x0, x1, x2, x3);
        distance::edge_edge_distance2(flag, x0, x1, x2, x3, D2);
    }
    return D2;
}

/****************************************************
*            FP32 DCD bound (alpha == 0)
****************************************************/

constexpr int DcdValid    = 1;  // finite exact distance > 0
constexpr int DcdWrong    = 2;  // rejected although D2 < expand^2
constexpr int DcdRejected = 4;  // rejected at half the exact distance

UIPC_DEVICE bool dcd_bound_rejects(bool point_triangle, const Vector3* x, Float dist)
{
    return point_triangle ?
               distance::point_triangle_farther_than(x[0], x[1], x[2], x[3], dist) :
               distance::edge_edge_farther_than(x[0], x[1], x[2], x[3], dist);
}

// The DCD branch of the AllP-AllT / AllE-AllE detect predicates: the FP32
// bound runs first and must never drop a pair whose FP64 test D2 < expand^2
// holds. expand is placed at, just above and well above the exact distance,
// where the FP64 test flips from rejecting to keeping.
__global__ void float_contact_lower_bound_dcd_kernel(const Vector3* cases,
                                                     int            n,
                                                     bool point_triangle,
                                                     int* outcomes)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    const Vector3* x  = cases + 4 * i;
    const Float    D2 = exact_distance2(point_triangle, x[0], x[1], x[2], x[3]);
    const Float    d  = sqrt(D2);
    if(!(d > 0.0) || !::isfinite(d))
    {
        outcomes[i] = 0;
        return;
    }
    int outcome = DcdValid;
    const Float factors[] = {1.0, 1.0 + 1e-12, 1.0 + 1e-9, 1.0 + 1e-6, 1.0 + 1e-3, 1.1, 2.0};
    for(Float f : factors)
    {
        const Float expand = d * f;
        if(D2 < expand * expand && dcd_bound_rejects(point_triangle, x, expand))
            outcome |= DcdWrong;
    }
    if(dcd_bound_rejects(point_triangle, x, 0.5 * d))
        outcome |= DcdRejected;
    outcomes[i] = outcome;
}

struct DcdStats
{
    SizeT valid            = 0;
    SizeT wrong            = 0;
    SizeT generic          = 0;  // valid cases of the generic generator branch
    SizeT generic_rejected = 0;
    int   first_wrong      = -1;
};

DcdStats run_dcd(bool point_triangle, int n, std::uint64_t seed, std::vector<Sample>& samples)
{
    Sampler s{seed};
    samples.clear();
    std::vector<Vector3> host;
    host.reserve(4 * SizeT(n));
    for(int i = 0; i < n; ++i)
    {
        samples.push_back(point_triangle ? sample_point_triangle(s) : sample_edge_edge(s));
        host.insert(host.end(), samples.back().x.begin(), samples.back().x.end());
    }

    cuda_tool::DeviceBuffer<Vector3> cases;
    cuda_tool::DeviceBuffer<int>     outcomes;
    cases.copy_from(host.data(), host.size());
    outcomes.resize_discard(n);
    constexpr int block = 128;
    float_contact_lower_bound_dcd_kernel<<<(n + block - 1) / block, block>>>(
        cases.data(), n, point_triangle, outcomes.data());
    CUDA_TOOL_CHECK(cudaGetLastError());
    std::vector<int> out;
    outcomes.copy_to(out);

    DcdStats stats;
    for(int i = 0; i < n; ++i)
    {
        if(!(out[i] & DcdValid))
            continue;
        ++stats.valid;
        if(out[i] & DcdWrong)
        {
            ++stats.wrong;
            if(stats.first_wrong < 0)
                stats.first_wrong = i;
        }
        if(samples[i].mode == 0)
        {
            ++stats.generic;
            stats.generic_rejected += (out[i] & DcdRejected) ? 1 : 0;
        }
    }
    return stats;
}

void check_dcd(bool point_triangle, std::uint64_t seed)
{
    constexpr int       n = 400000;
    std::vector<Sample> samples;
    DcdStats            stats = run_dcd(point_triangle, n, seed, samples);
    const std::string   first_wrong =
        stats.first_wrong < 0 ? std::string{"none"} :
                                  describe("pair", samples[stats.first_wrong].x);
    CAPTURE(stats.valid, stats.wrong, stats.generic, stats.generic_rejected);
    INFO("first wrong rejection: " << first_wrong);
    REQUIRE(stats.valid > SizeT(n) / 2);
    REQUIRE(stats.wrong == 0);
    // The bound is also active: on generic shapes it rejects many pairs at
    // half their distance (host-side measurement: 57% PT, 46% EE overall).
    REQUIRE(stats.generic_rejected * 4 > stats.generic);
}
}  // namespace

TEST_CASE("float_contact_lower_bound_dcd", "[cuda][collision detection][float_lower_bound]")
{
    SECTION("point-triangle")
    {
        check_dcd(true, 7);
    }
    SECTION("edge-edge")
    {
        check_dcd(false, 11);
    }
}

namespace
{
/****************************************************
*           FP32 swept bound (alpha > 0)
****************************************************/

// filter_toi's large_enough_toi and ACCD iteration limit.
constexpr Float LargeEnoughToi = 1.1;
constexpr int   AccdMaxIter    = 1000;
constexpr int   SweptSamples   = 64;

struct SweptParams
{
    Float alpha;  // line-search step: the step moves vertex k by alpha * dx_k
    Float theta;  // thickness = theta * expand, i.e. d_hat = (1 - theta) * expand
    Float size;    // characteristic size: absolute expand values scale with it
    Float t_star;  // constructed closest-approach time in [0, 1]
};

// Thresholds per case: relative to the smallest sampled distance (bits 0-5;
// the pair is kept where the factor exceeds 1) and absolute (bits 6-8).
constexpr int SweptRelative  = 6;
constexpr int SweptAbsolute  = 3;
constexpr int SweptUsefulBit = SweptRelative + 1;  // expand = 1e-2 * size

// The two primitives about ten sizes apart.
Sample sample_far(Sampler& s, bool point_triangle)
{
    const Vector3 o    = s.box(1e3);
    const Float   size = s.log_uniform(-4.0, 0.0);
    const Vector3 away = o + s.direction() * (10.0 * size);
    if(point_triangle)
        return {{away + s.box(0.5 * size), o + s.box(size), o + s.box(size), o + s.box(size)},
                size,
                0};
    return {{o + s.box(size), o + s.box(size), away + s.box(size), away + s.box(size)}, size, 0};
}

UIPC_DEVICE bool swept_bound_rejects(
    bool point_triangle, const Vector3* x, const Vector3* d, Float expand, Float thickness)
{
    return point_triangle ?
               distance::point_triangle_swept_farther_than(
                   x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], expand, thickness) :
               distance::edge_edge_swept_farther_than(
                   x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], expand, thickness);
}

// The same rejection on the FP32 inputs the trajectory filter prepares per
// vertex (swept_lower_bound_f32.h): absolute positions with d_hat in w, step
// displacements with the thickness in w; d_hat = expand - thickness, no skin.
UIPC_DEVICE bool swept_bound_rejects_f32(
    bool point_triangle, const Vector3* x, const Vector3* d, Float expand, Float thickness)
{
    float4 p[4], s[4];
    for(int k = 0; k < 4; ++k)
    {
        p[k] = make_float4(static_cast<float>(x[k].x()),
                           static_cast<float>(x[k].y()),
                           static_cast<float>(x[k].z()),
                           0.0f);
        s[k] = make_float4(static_cast<float>(d[k].x()),
                           static_cast<float>(d[k].y()),
                           static_cast<float>(d[k].z()),
                           0.0f);
    }
    const float d_hat = static_cast<float>(expand - thickness);
    s[0].w            = static_cast<float>(thickness);
    if(point_triangle)
    {
        p[0].w = p[1].w = d_hat;
        return distance::point_triangle_swept_farther_than_f32(
            p[0], p[1], p[2], p[3], s[0], s[1], s[2], s[3], 0.0f);
    }
    p[0].w = p[2].w = d_hat;
    return distance::edge_edge_swept_farther_than_f32(
        p[0], p[1], p[2], p[3], s[0], s[1], s[2], s[3], 0.0f);
}

// filter_toi for one PT/EE pair: the swept box test over d_hat + thickness,
// then ACCD from large_enough_toi.
UIPC_DEVICE Float filter_toi_reference(bool           point_triangle,
                                       const Vector3* x,
                                       const Vector3* d,
                                       Float          expand,
                                       Float          thickness,
                                       Float          eta)
{
    Float toi     = LargeEnoughToi;
    bool  hit     = false;
    bool  faraway = point_triangle ?
                        !distance::point_triangle_ccd_broadphase(
                           x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], expand) :
                        !distance::edge_edge_ccd_broadphase(
                           x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], expand);
    if(!faraway && point_triangle)
        hit = distance::point_triangle_ccd(
            x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], eta, thickness, AccdMaxIter, toi);
    else if(!faraway)
        hit = distance::edge_edge_ccd(
            x[0], x[1], x[2], x[3], d[0], d[1], d[2], d[3], eta, thickness, AccdMaxIter, toi);
    if(!hit)
        toi = LargeEnoughToi;
    return toi;
}

// The trajectory branch of the AllP-AllT / AllE-AllE detect predicates. A
// rejection by the swept bound is wrong when the pair comes within expand at
// some sampled time of the step (the FP64 distance at the end point is the
// accepted position that the next Newton iteration reuses), or when filter_toi
// would report a TOI below large_enough_toi for eta = 0.1 (IPC) or 0.001
// (AL-IPC). Sampling over-estimates the smallest distance, so it never flags
// a correct rejection.
__global__ void float_contact_lower_bound_swept_kernel(const Vector3* xs,
                                                       const Vector3* dxs,
                                                       const SweptParams* params,
                                                       int  n,
                                                       bool point_triangle,
                                                       bool f32_inputs,
                                                       int* wrong_masks,
                                                       int* reject_masks,
                                                       int* keep_masks)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    const Vector3*    x = xs + 4 * i;
    const SweptParams q = params[i];

    // Displacements as the detect predicate (step_displacement) and as
    // filter_toi (alpha * dx) form them.
    Vector3 d[4];
    Vector3 d_toi[4];
    for(int k = 0; k < 4; ++k)
    {
        d[k]     = step_displacement(q.alpha, dxs[4 * i + k]);
        d_toi[k] = q.alpha * dxs[4 * i + k];
    }

    Float dmin2 = -1.0;
    for(int sample = 0; sample <= SweptSamples + 1; ++sample)
    {
        const Float t = sample <= SweptSamples ? Float(sample) / SweptSamples : q.t_star;
        if(!(t >= 0.0 && t <= 1.0))
            continue;
        // t == 1 is bit-identical to step_position, the accepted position.
        Vector3 y[4];
        for(int k = 0; k < 4; ++k)
        {
            if(t == 1.0)
                y[k] = x[k] + d[k];
            else
                y[k] = x[k] + t * d[k];
        }
        const Float D2 = exact_distance2(point_triangle, y[0], y[1], y[2], y[3]);
        if(::isfinite(D2) && D2 >= 0.0 && (dmin2 < 0.0 || D2 < dmin2))
            dmin2 = D2;
    }

    const Float relative[SweptRelative] = {1.0 + 1e-9, 1.0 + 1e-6, 1.0 + 1e-3, 1.5, 0.5, 0.1};
    const Float absolute[SweptAbsolute] = {1e-3, 1e-2, 1e-1};
    const Float etas[]                  = {0.1, 0.001};

    int wrong = 0, rejected = 0, kept = 0;
    if(dmin2 >= 0.0)
    {
        const Float dmin = sqrt(dmin2);
        for(int e = 0; e < SweptRelative + SweptAbsolute; ++e)
        {
            const Float expand = e < SweptRelative ?
                                     dmin * relative[e] :
                                     q.size * absolute[e - SweptRelative];
            if(!(expand > 0.0))
                continue;
            const Float thickness = q.theta * expand;
            const int   bit       = 1 << e;
            const bool  keep      = dmin2 < expand * expand;
            kept |= keep ? bit : 0;
            if(!(f32_inputs ?
                     swept_bound_rejects_f32(point_triangle, x, d, expand, thickness) :
                     swept_bound_rejects(point_triangle, x, d, expand, thickness)))
                continue;
            rejected |= bit;
            bool bad = keep;
            for(Float eta : etas)
                bad = bad
                      || filter_toi_reference(point_triangle, x, d_toi, expand, thickness, eta)
                             < LargeEnoughToi;
            wrong |= bad ? bit : 0;
        }
    }
    wrong_masks[i]  = wrong;
    reject_masks[i] = rejected;
    keep_masks[i]   = kept;
}

struct SweptCase
{
    Sample      start;
    Simplex4    dx;
    SweptParams params;
    int         motion;
};

constexpr int MotionApproach = 0;  // target configuration at the end of the step
constexpr int MotionPass   = 1;    // target configuration at t_star
constexpr int MotionRandom = 2;    // independent vertex motion
constexpr int MotionFar    = 3;    // far apart, slow relative motion

// A target configuration (often a near contact) is placed at t_star; one
// primitive moves relative to the other by w, every vertex by a common T, and
// optionally by a small jitter.
SweptCase make_swept_case(Sampler& s, bool point_triangle)
{
    SweptCase c;
    c.motion           = s.pick(4);
    Sample      target = c.motion == MotionFar ? sample_far(s, point_triangle) :
                         point_triangle        ? sample_point_triangle(s) :
                                                 sample_edge_edge(s);
    const Float size   = target.size;
    const Float alpha  = s.log_uniform(-3.0, 0.0);
    const Float t_star = c.motion == MotionApproach ? 1.0 : s.uniform(0.0, 1.0);
    Vector3     T      = Vector3::Zero();
    if(s.uniform(0.0, 1.0) < 0.5)
        T = s.box(size * s.log_uniform(-2.0, 2.0));

    Simplex4 motion;
    switch(c.motion)
    {
        case MotionApproach:
        case MotionPass: {
            const Vector3 w = s.box(size * s.log_uniform(-4.0, 1.0));
            // PT: the point or the triangle moves; EE: the second edge moves.
            const int first = point_triangle ? (s.pick(2) == 0 ? 0 : 1) : 2;
            const int last  = point_triangle && first == 0 ? 1 : 4;
            for(int k = 0; k < 4; ++k)
            {
                motion[k] = T;
                if(k >= first && k < last)
                    motion[k] += w;
            }
            break;
        }
        case MotionRandom:
            for(int k = 0; k < 4; ++k)
                motion[k] = T + s.box(size * s.log_uniform(-6.0, 1.0));
            break;
        default:
            for(int k = 0; k < 4; ++k)
                motion[k] = T + s.box(0.05 * size);
            break;
    }
    if(s.uniform(0.0, 1.0) < 0.5)
        for(int k = 0; k < 4; ++k)
            motion[k] += s.box(size * s.log_uniform(-8.0, -1.0));

    c.start = target;
    for(int k = 0; k < 4; ++k)
    {
        c.start.x[k] = target.x[k] - t_star * motion[k];
        c.dx[k]      = motion[k] / alpha;
    }
    c.params = SweptParams{alpha, s.uniform(0.0, 0.9), size, t_star};
    return c;
}

struct SweptStats
{
    SizeT cases = 0;
    SizeT kept  = 0;  // thresholds at which the sampled distance keeps the pair
    SizeT rejected       = 0;  // thresholds at which the swept bound rejects
    SizeT wrong          = 0;
    SizeT apart          = 0;
    SizeT apart_rejected = 0;
    int   first_wrong    = -1;
};

int popcount(int v)
{
    int c = 0;
    for(; v; v &= v - 1)
        ++c;
    return c;
}

void check_swept(bool point_triangle, std::uint64_t seed, bool f32_inputs = false)
{
    constexpr int            n = 200000;
    Sampler                  s{seed};
    std::vector<SweptCase>   cases;
    std::vector<Vector3>     host_x, host_dx;
    std::vector<SweptParams> host_params;
    cases.reserve(n);
    for(int i = 0; i < n; ++i)
    {
        cases.push_back(make_swept_case(s, point_triangle));
        const SweptCase& c = cases.back();
        host_x.insert(host_x.end(), c.start.x.begin(), c.start.x.end());
        host_dx.insert(host_dx.end(), c.dx.begin(), c.dx.end());
        host_params.push_back(c.params);
    }

    cuda_tool::DeviceBuffer<Vector3>     xs;
    cuda_tool::DeviceBuffer<Vector3>     dxs;
    cuda_tool::DeviceBuffer<SweptParams> params;
    cuda_tool::DeviceBuffer<int>         wrong_masks;
    cuda_tool::DeviceBuffer<int>         reject_masks;
    cuda_tool::DeviceBuffer<int>         keep_masks;
    xs.copy_from(host_x.data(), host_x.size());
    dxs.copy_from(host_dx.data(), host_dx.size());
    params.copy_from(host_params.data(), host_params.size());
    wrong_masks.resize_discard(n);
    reject_masks.resize_discard(n);
    keep_masks.resize_discard(n);
    constexpr int block = 128;
    float_contact_lower_bound_swept_kernel<<<(n + block - 1) / block, block>>>(
        xs.data(),
        dxs.data(),
        params.data(),
        n,
        point_triangle,
        f32_inputs,
        wrong_masks.data(),
        reject_masks.data(),
        keep_masks.data());
    CUDA_TOOL_CHECK(cudaGetLastError());
    std::vector<int> wrong, rejected, kept;
    wrong_masks.copy_to(wrong);
    reject_masks.copy_to(rejected);
    keep_masks.copy_to(kept);

    SweptStats stats;
    stats.cases = n;
    for(int i = 0; i < n; ++i)
    {
        stats.kept += popcount(kept[i]);
        stats.rejected += popcount(rejected[i]);
        stats.wrong += popcount(wrong[i]);
        if(wrong[i] && stats.first_wrong < 0)
            stats.first_wrong = i;
        if(cases[i].motion == MotionFar)
        {
            ++stats.apart;
            stats.apart_rejected += (rejected[i] >> SweptUsefulBit) & 1;
        }
    }

    std::string first_wrong = "none";
    if(stats.first_wrong >= 0)
    {
        const SweptCase&   c = cases[stats.first_wrong];
        std::ostringstream os;
        os.precision(17);
        os << describe("start", c.start.x) << "; " << describe("dx", c.dx)
           << "; alpha " << c.params.alpha << ", theta " << c.params.theta
           << ", size " << c.params.size << ", t_star " << c.params.t_star
           << ", wrong threshold mask " << wrong[stats.first_wrong];
        first_wrong = os.str();
    }
    CAPTURE(
        stats.cases, stats.kept, stats.rejected, stats.wrong, stats.apart, stats.apart_rejected);
    INFO("first wrong rejection: " << first_wrong);
    REQUIRE(stats.wrong == 0);
    REQUIRE(stats.kept > 0);
    // Far, slowly moving pairs are what the bound exists for.
    REQUIRE(stats.apart_rejected * 2 > stats.apart);
}
}  // namespace

TEST_CASE("float_contact_lower_bound_swept", "[cuda][collision detection][swept_lower_bound]")
{
    SECTION("point-triangle")
    {
        check_swept(true, 13);
    }
    SECTION("edge-edge")
    {
        check_swept(false, 17);
    }
    SECTION("point-triangle, FP32 inputs")
    {
        check_swept(true, 13, true);
    }
    SECTION("edge-edge, FP32 inputs")
    {
        check_swept(false, 17, true);
    }
}

TEST_CASE("float_contact_lower_bound_coincident", "[cuda][collision detection][float_lower_bound]")
{
    using namespace uipc;
    using namespace uipc::backend::cuda;
    // Every point of the pair at one place (a fully collapsed triangle or
    // two collapsed edges): no bound may reject the pair, which goes to the
    // exact test.
    const Vector3 x{0.25, -1.5, 3.0};
    const Vector3 zero = Vector3::Zero();
    const float4  f{0.25f, -1.5f, 3.0f, 1e-3f};
    const float4  df{0.0f, 0.0f, 0.0f, 0.0f};
    for(Float dist : {0.0, 1e-3})
    {
        CHECK_FALSE(distance::point_triangle_farther_than(x, x, x, x, dist));
        CHECK_FALSE(distance::edge_edge_farther_than(x, x, x, x, dist));
        CHECK_FALSE(distance::point_triangle_swept_farther_than(
            x, x, x, x, zero, zero, zero, zero, dist, 0.0));
        CHECK_FALSE(distance::edge_edge_swept_farther_than(
            x, x, x, x, zero, zero, zero, zero, dist, 0.0));
    }
    for(float skin : {0.0f, 1e-3f})
    {
        CHECK_FALSE(distance::point_triangle_swept_farther_than_f32(
            f, f, f, f, df, df, df, df, skin));
        CHECK_FALSE(distance::edge_edge_swept_farther_than_f32(f, f, f, f, df, df, df, df, skin));
    }
}
