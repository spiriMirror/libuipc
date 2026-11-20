/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
// Warp-cooperative projection of a small symmetric matrix onto the PSD cone
// (negative eigenvalues clamped to zero), for kernels that give one warp to
// one problem.
//
// On GB20x FP64 issues per warp instruction whatever the active lanes, and a
// dependent DFMA chain runs at about 19 ns per instruction, so a thread-serial
// 12x12 eigensolver costs its full instruction count (Eigen's make_spd<12> is
// about 350 us on one warp). Here the lanes share the matrix work of the
// classical cyclic Jacobi method in its parallel ordering: each round applies
// the N/2 disjoint rotations of a round-robin pairing; lanes compute the
// rotations of different pairs at once and then update different entries of A
// and V.
//
// Everything is FP64. The rotation tangent t (the small root of
// t^2 + 2 theta t - 1 = 0) is solved to full double precision by two Newton
// steps started from an FP32 estimate (no double division or square root on
// the critical path); the annihilated entry is then set to zero and the two
// diagonal entries updated as app - t apq, aqq + t apq (Rutishauser), as in
// the textbook algorithm.
// The matrix is scaled by a power of two (exactly) to a Frobenius norm in
// [1, 2). The sweeps stop when every off-diagonal entry is at most 2e-15 of
// the norm or after 24 sweeps; there is no early stop on a slow sweep (nearly
// degenerate clusters, e.g. the ortho-potential Hessian near a rotation,
// converge linearly for a few sweeps first).
//
// From the fifth sweep on, a rotation is skipped (the entry set to zero
// instead) when a_pq is negligible against the diagonal by Demmel and
// Veselic's relative criterion |a_pq| <= u sqrt(|a_pp a_qq|), u = 2^-53 the
// unit roundoff (Rutishauser's skip of negligible entries, with this
// threshold). Zeroing such an entry moves A by at most one rounding of a
// diagonal entry, and the projection, being 1-Lipschitz, by no more. Without
// the skip, exactly degenerate clusters converge slowly: the revolute joint
// angle Hessian (external articulation, revolute joint limit) has eigenvalues
// +-1/(2 rho), each about 6 times, whose diagonal entries soon agree to the
// last bit; a rounding-sized a_pq between two of them then drives a 45 degree
// rotation that mixes cross-cluster couplings not yet annihilated back into
// annihilated slots, so those couplings only halve per sweep, and such an
// eigensolve can reach the 24-sweep cap before full accuracy; with the skip
// they converge in a few sweeps and are on average more accurate.
// Matrices without such clusters are not affected (the skip rarely triggers;
// when it does, the result moves by rounding).
//
// The projection is assembled from the side with the smaller eigenvalue mass:
// P = V max(L, 0) V^T, or P = A - V min(L, 0) V^T when the negative part is
// the smaller one (the usual case for the nearly PSD joint and ortho
// Hessians, whose projection then keeps A's own rounding). A is read as
// symmetric from its lower triangle (as Eigen's SelfAdjointEigenSolver), and P
// is written exactly symmetric.
//
// Layout: A, A2, V, V2 are N x N column-major arrays in shared memory owned by
// the calling warp; rot holds warp_spd_rot_size<N> Floats and part N ints.
// A is destroyed (A2, V, V2 too). Every lane of the warp must call the
// function (it synchronizes the warp). On return P (N x N; must not alias the
// work arrays) holds the projection.
//
// The result equals make_spd() up to rounding (the [warp_spd] tests measure
// both against a long double eigendecomposition).
template <int N>
constexpr int warp_spd_rot_size = 3 * N;

namespace warp_spd_detail
{
    // Round-robin schedule over M (even) players: player M - 1 keeps
    // position M - 1, player x < M - 1 sits at position (x - r) mod (M - 1)
    // in round r; positions k and M - 1 - k are paired.
    template <int M>
    UIPC_DEVICE inline int player_at(int pos, int r)
    {
        return pos == M - 1 ? M - 1 : (pos + r) % (M - 1);
    }

    // Jacobi rotation J = [[c, s], [-s, c]] on (p, q) with (J^T A J)(p, q) = 0
    // for the 2x2 block (app, apq; apq, aqq), apq != 0: t = s / c is the root
    // of g(t) = w t^2 + 2 d t - w (d = aqq - app, w = 2 apq) with |t| <= 1.
    // d and w are first scaled by a power of two to max(|d|, |w|) in [1, 2),
    // so the FP32 estimate neither overflows nor loses the ratio.
    UIPC_DEVICE inline void jacobi_rotation(
        Float app, Float apq, Float aqq, Float& c, Float& s, Float& t)
    {
        // ilogb / scalbn without their special-case branches when they are
        // not needed: for a normal m with |e| <= 216 the library scalbn is one
        // multiplication by the exact 2^-e (it splits larger shifts over
        // several factors), so the direct product below is bitwise the same,
        // also for an underflowing result; anything else takes the library.
        const Float m = fmax(fabs(aqq - app), fabs(2 * apq));
        const int biased = static_cast<int>((__double_as_longlong(m) >> 52) & 0x7ff);
        const int e = biased - 1023;
        Float     d, w;
        if(biased != 0 && e >= -216 && e <= 216)
        {
            const Float scale =
                __longlong_as_double(static_cast<long long>(1023 - e) << 52);
            d = (aqq - app) * scale;
            w = (2 * apq) * scale;
        }
        else
        {
            const int el = ilogb(m);
            d            = scalbn(aqq - app, -el);
            w            = scalbn(2 * apq, -el);
        }
        // FP32 estimate: t = sgn(d) w / (|d| + sqrt(d^2 + w^2))
        const float df = static_cast<float>(d);
        const float wf = static_cast<float>(w);
        const float rf = sqrtf(df * df + wf * wf);
        Float x = static_cast<Float>(copysignf(1.0f, df) * wf / (fabsf(df) + rf));
        // Two Newton steps on g with an FP32 reciprocal of g'(x) = 2 (w x + d)
        // (bounded away from zero at the small root): the first leaves about
        // 1e-14, the second full double precision.
        const Float r =
            static_cast<Float>(__frcp_rn(2.0f * static_cast<float>(fma(w, x, d))));
        x = x - fma(fma(w, x, 2 * d), x, -w) * r;
        x = x - fma(fma(w, x, 2 * d), x, -w) * r;
        t = x;
        // c = 1 / sqrt(1 + t^2): FP32 rsqrt refined by two Newton steps
        const Float y  = fma(t, t, Float(1));
        Float       c0 = static_cast<Float>(rsqrtf(static_cast<float>(y)));
        c0             = c0 * fma(Float(-0.5) * y, c0 * c0, Float(1.5));
        c0             = c0 * fma(Float(-0.5) * y, c0 * c0, Float(1.5));
        c              = c0;
        s              = t * c0;
    }

    // a_pq negligible: |a_pq| <= 2^-53 sqrt(|a_pp a_qq|), compared squared
    // (the matrix is scaled to a norm in [1, 2): no overflow; an underflowing
    // product only affects entries below 1e-150 of the norm)
    UIPC_DEVICE inline bool negligible(Float app, Float apq, Float aqq)
    {
        return __dmul_rn(apq, apq) <= __dmul_rn(0x1p-106, fabs(__dmul_rn(app, aqq)));
    }
}  // namespace warp_spd_detail

// Returns the number of sweeps run (24 when the tolerance was not reached).
template <int N>
UIPC_DEVICE int warp_make_spd(
    Float* A, Float* A2, Float* V, Float* V2, Float* rot, int* part, Float* P, int lane)
{
    constexpr int M = (N + 1) / 2 * 2;  // players (one phantom when N is odd)
    constexpr int Rounds    = M - 1;
    constexpr int NN        = N * N;
    constexpr int PerLane   = (NN + 31) / 32;
    constexpr int MaxSweeps = 24;
    constexpr int SkipFrom = 4;  // negligible entries are skipped from the fifth sweep on
    constexpr Float Tol = 2e-15;  // on off-diagonal entries, relative to the Frobenius norm

    // symmetric input from the lower triangle, kept in P; V = I
#pragma unroll
    for(int k = 0; k < PerLane; ++k)
    {
        const int e = lane + 32 * k;
        if(e < NN)
        {
            const int i = e % N;
            const int j = e / N;
            P[e]        = i >= j ? A[e] : A[j + i * N];
        }
    }
    __syncwarp();

    // scale by a power of two (exact) to a Frobenius norm in [1, 2)
    Float local = 0;
#pragma unroll
    for(int k = 0; k < PerLane; ++k)
    {
        const int e = lane + 32 * k;
        if(e < NN)
            local += P[e] * P[e];
    }
#pragma unroll
    for(int off = 16; off > 0; off >>= 1)
        local += __shfl_xor_sync(0xffffffffu, local, off);
    const int   exponent = local > 0 ? ilogb(sqrt(local)) : 0;
    const Float tol      = Tol * scalbn(sqrt(local), -exponent);
#pragma unroll
    for(int k = 0; k < PerLane; ++k)
    {
        const int e = lane + 32 * k;
        if(e < NN)
        {
            A[e] = scalbn(P[e], -exponent);
            V[e] = (e % N == e / N) ? Float(1) : Float(0);
        }
    }
    __syncwarp();

    int sweeps = MaxSweeps;
    for(int sweep = 0; sweep < MaxSweeps; ++sweep)
    {
        Float off = 0;
#pragma unroll
        for(int k = 0; k < PerLane; ++k)
        {
            const int e = lane + 32 * k;
            if(e < NN && e % N != e / N)
                off = fmax(off, fabs(A[e]));
        }
#pragma unroll
        for(int o = 16; o > 0; o >>= 1)
            off = fmax(off, __shfl_xor_sync(0xffffffffu, off, o));
        if(off <= tol)
        {
            sweeps = sweep;
            break;
        }

        for(int r = 0; r < Rounds; ++r)
        {
            // rot[3 i] = c_i, rot[3 i + 1] = sigma_i (column i of A J is
            // c_i a_i + sigma_i a_part(i)), rot[3 i + 2] = the exact change of
            // the diagonal entry i
            if(lane < M / 2)
            {
                const int a = warp_spd_detail::player_at<M>(lane, r);
                const int b = warp_spd_detail::player_at<M>(M - 1 - lane, r);
                const int p = min(a, b);
                const int q = max(a, b);
                if(q < N)
                {
                    const Float app = A[p + p * N];
                    const Float apq = A[p + q * N];
                    const Float aqq = A[q + q * N];
                    Float       c = 1, s = 0, t = 0;
                    if(apq != 0
                       && !(sweep >= SkipFrom && warp_spd_detail::negligible(app, apq, aqq)))
                        warp_spd_detail::jacobi_rotation(app, apq, aqq, c, s, t);
                    rot[3 * p]     = c;
                    rot[3 * p + 1] = -s;
                    rot[3 * p + 2] = -t * apq;
                    rot[3 * q]     = c;
                    rot[3 * q + 1] = s;
                    rot[3 * q + 2] = t * apq;
                    part[p]        = q;
                    part[q]        = p;
                }
                else if(p < N)  // paired with the phantom: untouched
                {
                    rot[3 * p]     = 1;
                    rot[3 * p + 1] = 0;
                    rot[3 * p + 2] = 0;
                    part[p]        = p;
                }
            }
            __syncwarp();

            // A2 = J^T A J, V2 = V J
#pragma unroll
            for(int k = 0; k < PerLane; ++k)
            {
                const int e = lane + 32 * k;
                if(e < NN)
                {
                    const int   i  = e % N;
                    const int   j  = e / N;
                    const int   pi = part[i];
                    const int   pj = part[j];
                    const Float ci = rot[3 * i], si = rot[3 * i + 1];
                    const Float cj = rot[3 * j], sj = rot[3 * j + 1];
                    // every lane evaluates both forms and selects, instead of
                    // branching per entry (the lanes of a warp take different
                    // branches); the diagonal keeps Rutishauser's update and
                    // the annihilated entry its zero
                    const Float x0 = ci * A[i + j * N] + si * A[pi + j * N];
                    const Float x1 = ci * A[i + pj * N] + si * A[pi + pj * N];
                    const Float rotated  = cj * x0 + sj * x1;
                    const Float diagonal = A[e] + rot[3 * i + 2];
                    A2[e] = i == j ? diagonal : (pi == j ? Float(0) : rotated);
                    V2[e] = cj * V[i + j * N] + sj * V[i + pj * N];
                }
            }
            __syncwarp();
            Float* t = A;
            A        = A2;
            A2       = t;
            t        = V;
            V        = V2;
            V2       = t;
        }
    }

    // eigenvalue mass on each side of zero (every lane, from the diagonal)
    Float positive = 0, negative = 0;
#pragma unroll
    for(int m = 0; m < N; ++m)
    {
        const Float lambda = A[m + m * N];
        positive += lambda > 0 ? lambda : Float(0);
        negative += lambda < 0 ? -lambda : Float(0);
    }
    const bool subtract = negative < positive;

    // P = V max(L, 0) V^T, or A - V min(L, 0) V^T; each pair (i <= j) by one
    // lane, mirrored
#pragma unroll
    for(int k = 0; k < PerLane; ++k)
    {
        const int e = lane + 32 * k;
        if(e < NN)
        {
            const int i = e % N;
            const int j = e / N;
            if(i <= j)
            {
                Float p = 0;
#pragma unroll
                for(int m = 0; m < N; ++m)
                {
                    const Float lambda = A[m + m * N];
                    const Float w = subtract ? (lambda < 0 ? lambda : Float(0)) :
                                               (lambda > 0 ? lambda : Float(0));
                    p = fma(V[i + m * N] * w, V[j + m * N], p);
                }
                const Float value =
                    subtract ? P[e] - scalbn(p, exponent) : scalbn(p, exponent);
                P[e]         = value;
                P[j + i * N] = value;
            }
        }
    }
    __syncwarp();
    return sweeps;
}
}  // namespace uipc::backend::cuda
