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

#include <app/app.h>

#include <utils/warp_spd.h>
#include <utils/make_spd.h>
#include <utils/fixed_bank_soa_evd.h>
#include "long_double_reference.h"
#include "eigen_reference.h"
#include <utils/warp_joint_hessian.h>
#include <cuda_tool/stream.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>
#include <affine_body/constraints/external_articulation_constraint_function.h>
#include <affine_body/constitutions/affine_body_prismatic_joint_function.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
template <int N>
__global__ void warp_spd_test_kernel(const Float* input, Float* output, int* sweeps, int count)
{
    const int I    = blockIdx.x;
    const int lane = threadIdx.x;
    if(I >= count)
        return;
    __shared__ Float A[N * N], A2[N * N], V[N * N], V2[N * N], P[N * N];
    __shared__ Float rot[warp_spd_rot_size<N>];
    __shared__ int   part[N];
    for(int e = lane; e < N * N; e += 32)
        A[e] = input[I * N * N + e];
    __syncwarp();
    const int s = warp_make_spd<N>(A, A2, V, V2, rot, part, P, lane);
    for(int e = lane; e < N * N; e += 32)
        output[I * N * N + e] = P[e];
    if(lane == 0)
        sweeps[I] = s;
}

// The projections the warp kernels replace: Eigen's thread-serial make_spd
// (revolute limit, prismatic joint and limit) and the fixed-bank shared-memory
// variant (external articulation, ortho potential).
template <int N>
__global__ void eigen_spd_test_kernel(const Float* input, Float* output, int count)
{
    const int I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= count)
        return;
    Matrix<Float, N, N> H = Eigen::Map<const Matrix<Float, N, N>>(input + I * N * N);
    make_spd(H);
    for(int e = 0; e < N * N; ++e)
        output[I * N * N + e] = H.data()[e];
}

template <int N>
__global__ void fixed_bank_spd_test_kernel(const Float* input, Float* output, int count)
{
    constexpr int    Pitch = 16;
    __shared__ Float workspace[N * N * Pitch];
    const int        I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= count)
        return;
    FixedBankSoAMap<N, Pitch> H(workspace + threadIdx.x);
    for(int e = 0; e < N * N; ++e)
        H(e % N, e / N) = input[I * N * N + e];
    Matrix<Float, N, N> P;
    make_spd_fixed_bank_shared_upper_fma<N>(H, P);
    for(int e = 0; e < N * N; ++e)
        output[I * N * N + e] = P.data()[e];
}

template <typename Kernel>
std::vector<Float> run_spd_kernel(Kernel k, const std::vector<Float>& input, int grid, int block, int count)
{
    Float *d_in = nullptr, *d_out = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&d_in, input.size() * sizeof(Float)));
    CUDA_TOOL_CHECK(cudaMalloc(&d_out, input.size() * sizeof(Float)));
    CUDA_TOOL_CHECK(cudaMemcpy(d_in, input.data(), input.size() * sizeof(Float), cudaMemcpyHostToDevice));
    k<<<grid, block>>>(d_in, d_out, count);
    CUDA_TOOL_CHECK(cudaGetLastError());
    std::vector<Float> output(input.size());
    CUDA_TOOL_CHECK(cudaMemcpy(output.data(), d_out, output.size() * sizeof(Float), cudaMemcpyDeviceToHost));
    cudaFree(d_in);
    cudaFree(d_out);
    return output;
}

// A warp projection kernel on `count` N x N matrices; `sweeps` receives the
// Jacobi sweeps each one took.
template <int N, typename Kernel>
std::vector<Float> run_warp_kernel(Kernel                    k,
                                   const std::vector<Float>& input,
                                   int                       count,
                                   std::vector<int>&         sweeps)
{
    Float *d_in = nullptr, *d_out = nullptr;
    int*   d_sweeps = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&d_in, input.size() * sizeof(Float)));
    CUDA_TOOL_CHECK(cudaMalloc(&d_out, input.size() * sizeof(Float)));
    CUDA_TOOL_CHECK(cudaMalloc(&d_sweeps, count * sizeof(int)));
    CUDA_TOOL_CHECK(cudaMemcpy(d_in, input.data(), input.size() * sizeof(Float), cudaMemcpyHostToDevice));
    k<<<count, 32>>>(d_in, d_out, d_sweeps, count);
    CUDA_TOOL_CHECK(cudaGetLastError());
    std::vector<Float> output(input.size());
    sweeps.resize(count);
    CUDA_TOOL_CHECK(cudaMemcpy(output.data(), d_out, output.size() * sizeof(Float), cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(sweeps.data(), d_sweeps, count * sizeof(int), cudaMemcpyDeviceToHost));
    cudaFree(d_in);
    cudaFree(d_out);
    cudaFree(d_sweeps);
    return output;
}

template <int N>
std::vector<Float> run_warp_spd(const std::vector<Float>& input, int count, std::vector<int>& sweeps)
{
    return run_warp_kernel<N>(warp_spd_test_kernel<N>, input, count, sweeps);
}

// Maximum Jacobi sweeps of warp_make_spd (the cap: 24).
constexpr int WarpSpdMaxSweeps = 24;

// The [warp_spd] test matrices: random, rank-deficient, clustered and
// repeated, diagonal, two nearly degenerate clusters (I % 4, I % 8 == 7).
template <int N>
std::vector<Float> warp_spd_test_inputs(int count, Float amplitude)
{
    using MatrixN = Eigen::Matrix<Float, N, N>;
    std::mt19937_64                       gen(0x7a5d0000ULL + N);
    std::uniform_real_distribution<Float> u(-1.0, 1.0);

    std::vector<Float> input(count * N * N);
    for(int I = 0; I < count; ++I)
    {
        MatrixN M;
        for(int i = 0; i < N; ++i)
            for(int j = 0; j <= i; ++j)
                M(i, j) = M(j, i) = amplitude * u(gen);
        if(I % 4 == 1)  // rank deficient
        {
            Eigen::Matrix<Float, N, 3> B =
                Eigen::Matrix<Float, N, 3>::NullaryExpr([&] { return u(gen); });
            M = amplitude * B * B.transpose();
        }
        if(I % 4 == 2)  // clustered and repeated eigenvalues
        {
            const MatrixN A = MatrixN::NullaryExpr([&] { return u(gen); });
            MatrixN       Q;
            uipc::test_reference::householder_q(N, A.data(), Q.data());
            Eigen::Vector<Float, N> l;
            for(int i = 0; i < N; ++i)
                l(i) = amplitude
                       * (i % 3 == 0 ? -1.0 : (i % 3 == 1 ? 1.0 : 1.0 + 1e-9 * i));
            M = Q * l.asDiagonal() * Q.transpose();
            M = (0.5 * (M + M.transpose())).eval();
        }
        if(I % 4 == 3)  // diagonal
        {
            M.setZero();
            for(int i = 0; i < N; ++i)
                M(i, i) = amplitude * u(gen);
        }
        if(I % 8 == 7)  // two nearly degenerate clusters (as the ortho-potential
                        // Hessian near a rotation: 0 x N/3 and 8 x 2N/3, split
        // by 1e-10 of the scale, some zero modes slightly negative)
        {
            const MatrixN A = MatrixN::NullaryExpr([&] { return u(gen); });
            MatrixN       Q;
            uipc::test_reference::householder_q(N, A.data(), Q.data());
            Eigen::Vector<Float, N> l;
            for(int i = 0; i < N; ++i)
                l(i) = amplitude * ((i < N / 3 ? 0.0 : 8.0) + 1e-10 * u(gen));
            M = Q * l.asDiagonal() * Q.transpose();
            M = (0.5 * (M + M.transpose())).eval();
        }
        for(int e = 0; e < N * N; ++e)
            input[I * N * N + e] = M.data()[e];
    }
    return input;
}

template <int N>
void check_warp_spd(int count, Float amplitude)
{
    using MatrixN          = Eigen::Matrix<Float, N, N>;
    using VectorN          = Eigen::Matrix<Float, N, 1>;
    const auto       input = warp_spd_test_inputs<N>(count, amplitude);
    std::vector<int> sweeps;
    const auto       output = run_warp_spd<N>(input, count, sweeps);
    const auto       eigen =
        run_spd_kernel(eigen_spd_test_kernel<N>, input, (count + 31) / 32, 32, count);
    const auto fixed =
        run_spd_kernel(fixed_bank_spd_test_kernel<N>, input, (count + 15) / 16, 16, count);

    // Accuracy against an 80-bit (long double) eigendecomposition on the host:
    // error = max |P - P_ref| / |M|_F per matrix, for the warp Jacobi
    // projection and the two projections it replaces.
    const auto reference =
        uipc::test_reference::psd_projection_long_double(N, input, count);
    // checks at the rounding level of double need a wider reference
    const bool extended = uipc::test_reference::long_double_is_extended();
    Float      worst    = 0;
    double     sum_w = 0, sum_e = 0, sum_f = 0, max_w = 0, max_e = 0, max_f = 0;
    for(int I = 0; I < count; ++I)
    {
        MatrixN M = Eigen::Map<const MatrixN>(input.data() + I * N * N);
        VectorN eigenvalues;
        MatrixN eigenvectors;
        REQUIRE(uipc::test_reference::symmetric_eigen(
            N, M.data(), eigenvalues.data(), eigenvectors.data()));
        MatrixN P = eigenvectors * eigenvalues.cwiseMax(0.0).asDiagonal()
                    * eigenvectors.transpose();
        MatrixN G = Eigen::Map<const MatrixN>(output.data() + I * N * N);
        REQUIRE(G.allFinite());
        const Float scale = std::max(M.norm(), Float(1e-300));
        worst = std::max(worst, (G - P).cwiseAbs().maxCoeff() / scale);
        // symmetric and positive semidefinite to rounding
        REQUIRE((G - G.transpose()).cwiseAbs().maxCoeff() <= 1e-14 * scale);
        VectorN G_eigenvalues;
        REQUIRE(uipc::test_reference::symmetric_eigen(N, G.data(), G_eigenvalues.data()));
        REQUIRE(G_eigenvalues.minCoeff() >= -1e-13 * scale);

        auto err = [&](const std::vector<Float>& out)
        {
            return uipc::test_reference::max_error_long_double(
                out, reference, I * N * N, N * N, scale);
        };
        const double ew = err(output), ee = err(eigen), ef = err(fixed);
        // Every projection reaches the tolerance before the sweep cap, and
        // (nearly) degenerate clusters converge to full accuracy instead of
        // stopping in their linear phase (a stop on a slow sweep left 1e-5
        // here; without the negligible-entry skip some clustered matrices ran
        // to the cap).
        INFO("matrix " << I << ": " << sweeps[I] << " sweeps, error " << ew);
        CHECK(sweeps[I] < WarpSpdMaxSweeps);
        if(extended && (I % 4 == 2 || I % 8 == 7))
            CHECK(ew <= 2e-15);
        sum_w += ew;
        sum_e += ee;
        sum_f += ef;
        max_w = std::max(max_w, ew);
        max_e = std::max(max_e, ee);
        max_f = std::max(max_f, ef);
    }
    std::printf(
        "warp_spd accuracy N=%d amplitude=%.0e: max|P - P_ref|/|M|_F mean / max: "
        "warp Jacobi %.3e / %.3e, Eigen make_spd %.3e / %.3e, fixed-bank %.3e / %.3e\n",
        N,
        amplitude,
        sum_w / count,
        max_w,
        sum_e / count,
        max_e,
        sum_f / count,
        max_f);
    INFO("N = " << N << ", amplitude = " << amplitude
                << ", worst |P_warp - P_eigen| / |M| = " << worst);
    CHECK(worst <= 1e-13);
    // Not less accurate than the projections it replaces (with a margin for
    // the sampling noise of rounding-level errors).
    const double replaced_mean = std::min(sum_e, sum_f) / count;
    const double replaced_max  = std::min(max_e, max_f);
    if(extended)
    {
        CHECK(sum_w / count <= 1.5 * replaced_mean + 1e-18);
        CHECK(max_w <= 3.0 * replaced_max + 1e-17);
    }
}

template <bool Translation>
__global__ void basis_jt_h_j_test_kernel(const Float* H_F, const Vector6* bases, Float* out)
{
    const int lane = threadIdx.x;
    for(int e = lane; e < 576; e += 32)
        out[e] = basis_jt_h_j_entry<Translation>(
            H_F, bases[0], bases[1], bases[2], bases[3], e % 24, e / 24);
}

// map 0: revolute joint limit, 1: external revolute, 2: external prismatic
__global__ void sym_jt_h_j_test_kernel(const Float* H_F, const Vector6* bases, Float* out, int map)
{
    Matrix12x12                  H = Eigen::Map<const Matrix12x12>(H_F);
    Eigen::Matrix<Float, 24, 24> R;
    if(map == 0)
        sym::affine_body_revolute_joint_limit::JT_H_J<Float>(
            R, H, bases[0], bases[1], bases[2], bases[3]);
    else if(map == 1)
        sym::external_revolute_joint_constraint::JT_H_J<Float>(
            R, H, bases[0], bases[1], bases[2], bases[3]);
    else
        sym::external_prismatic_joint_constraint::JT_H_J<Float>(
            R, H, bases[0], bases[1], bases[2], bases[3]);
    for(int e = 0; e < 576; ++e)
        out[e] = R(e % 24, e / 24);
}
}  // namespace

TEST_CASE("warp Jacobi PSD projection matches Eigen make_spd", "[warp_spd]")
{
    for(Float amplitude : {1e-6, 1.0, 1e6})
    {
        check_warp_spd<12>(256, amplitude);
        check_warp_spd<9>(256, amplitude);
    }
}

namespace
{
// The revolute joint angle Hessian d2 DeltaTheta / dF2 that the external
// articulation (map 1) and revolute joint limit (map 0) kernels project,
// scaled by G_theta or dE/dx: for orthonormal joint frames its eigenvalues
// are +-1/2, six times each, and a frame deformation splits them only
// slightly (exactly degenerate clusters).
__global__ void revolute_angle_hessian_kernel(
    const Vector12* F, const Vector12* F_t, const Float* scale, Float* H, int count, int map)
{
    const int I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= count)
        return;
    Matrix12x12 h;
    if(map == 0)
        sym::affine_body_revolute_joint_limit::ddDeltaTheta_ddF<Float>(h, F[I], F_t[I]);
    else
        sym::external_revolute_joint_constraint::ddDeltaTheta_ddF<Float>(h, F[I], F_t[I]);
    for(int e = 0; e < 144; ++e)
        H[I * 144 + e] = scale[I] * h(e % 12, e / 12);
}
}  // namespace

TEST_CASE("warp Jacobi PSD projection converges on the revolute joint angle Hessian", "[warp_spd]")
{
    using Matrix3                               = Eigen::Matrix<Float, 3, 3>;
    using Vec3                                  = Eigen::Vector<Float, 3>;
    constexpr int                         count = 256;
    std::mt19937_64                       gen(0x7a5d3333ULL);
    std::normal_distribution<Float>       nd;
    std::uniform_real_distribution<Float> u(-3.0, 3.0);
    // F = (b_k, n_k, b_l, n_l): the frame (b, n) normal to the joint axis on
    // body k, and on body l rotated by the joint angle, in a random world
    // orientation, plus a deformation of size `noise`
    auto frame = [&](Float noise)
    {
        Vec3    axis = Vec3(nd(gen), nd(gen), nd(gen)).normalized();
        Vec3    b    = axis.cross(Vec3(nd(gen), nd(gen), nd(gen))).normalized();
        Vec3    n    = axis.cross(b);
        Matrix3 R    = Eigen::AngleAxis<Float>(u(gen), axis).toRotationMatrix();
        Matrix3 G = Eigen::Quaternion<Float>(nd(gen), nd(gen), nd(gen), nd(gen))
                        .normalized()
                        .toRotationMatrix();
        Vector12 f;
        f << G * b, G * n, G * R * b, G * R * n;
        for(int k = 0; k < 12; ++k)
            f(k) += noise * nd(gen);
        return f;
    };
    for(Float noise : {0.0, 1e-9, 1e-6, 1e-3})
    {
        for(int map = 0; map < 2; ++map)
        {
            std::vector<Vector12> F(count), F_t(count);
            std::vector<Float>    scale(count);
            for(int I = 0; I < count; ++I)
            {
                F[I]     = frame(noise);
                F_t[I]   = frame(noise);
                scale[I] = (I % 2 ? 1.0 : -1.0) * std::pow(10.0, u(gen));
            }
            Vector12 *d_F = nullptr, *d_Ft = nullptr;
            Float *   d_scale = nullptr, *d_H = nullptr;
            CUDA_TOOL_CHECK(cudaMalloc(&d_F, count * sizeof(Vector12)));
            CUDA_TOOL_CHECK(cudaMalloc(&d_Ft, count * sizeof(Vector12)));
            CUDA_TOOL_CHECK(cudaMalloc(&d_scale, count * sizeof(Float)));
            CUDA_TOOL_CHECK(cudaMalloc(&d_H, count * 144 * sizeof(Float)));
            CUDA_TOOL_CHECK(cudaMemcpy(d_F, F.data(), count * sizeof(Vector12), cudaMemcpyHostToDevice));
            CUDA_TOOL_CHECK(cudaMemcpy(d_Ft, F_t.data(), count * sizeof(Vector12), cudaMemcpyHostToDevice));
            CUDA_TOOL_CHECK(cudaMemcpy(d_scale, scale.data(), count * sizeof(Float), cudaMemcpyHostToDevice));
            revolute_angle_hessian_kernel<<<(count + 31) / 32, 32>>>(
                d_F, d_Ft, d_scale, d_H, count, map);
            CUDA_TOOL_CHECK(cudaGetLastError());
            std::vector<Float> input(count * 144);
            CUDA_TOOL_CHECK(cudaMemcpy(input.data(), d_H, input.size() * sizeof(Float), cudaMemcpyDeviceToHost));
            cudaFree(d_F);
            cudaFree(d_Ft);
            cudaFree(d_scale);
            cudaFree(d_H);

            std::vector<int> sweeps;
            const auto       output = run_warp_spd<12>(input, count, sweeps);
            const auto       eigen  = run_spd_kernel(
                eigen_spd_test_kernel<12>, input, (count + 31) / 32, 32, count);
            const auto fixed = run_spd_kernel(
                fixed_bank_spd_test_kernel<12>, input, (count + 15) / 16, 16, count);
            const auto reference =
                uipc::test_reference::psd_projection_long_double(12, input, count);
            const bool extended = uipc::test_reference::long_double_is_extended();
            double sum_w = 0, sum_e = 0, sum_f = 0, max_w = 0, max_e = 0, max_f = 0;
            int max_sweeps = 0;
            for(int I = 0; I < count; ++I)
            {
                Matrix12x12 M = Eigen::Map<const Matrix12x12>(input.data() + I * 144);
                Matrix12x12 G = Eigen::Map<const Matrix12x12>(output.data() + I * 144);
                REQUIRE(G.allFinite());
                const Float scale_I = std::max(M.norm(), Float(1e-300));
                REQUIRE((G - G.transpose()).cwiseAbs().maxCoeff() <= 1e-14 * scale_I);
                auto err = [&](const std::vector<Float>& out)
                {
                    return uipc::test_reference::max_error_long_double(
                        out, reference, I * 144, 144, scale_I);
                };
                const double ew = err(output), ee = err(eigen), ef = err(fixed);
                // converged before the cap and to full accuracy (without the
                // negligible-entry skip the eigensolver can run such Hessians
                // to the 24-sweep cap and stop short of full accuracy)
                INFO("noise " << noise << ", map " << map << ", matrix " << I
                              << ": " << sweeps[I] << " sweeps, error " << ew);
                CHECK(sweeps[I] < WarpSpdMaxSweeps);
                if(extended)
                    CHECK(ew <= 2e-15);
                sum_w += ew;
                sum_e += ee;
                sum_f += ef;
                max_w      = std::max(max_w, ew);
                max_e      = std::max(max_e, ee);
                max_f      = std::max(max_f, ef);
                max_sweeps = std::max(max_sweeps, sweeps[I]);
            }
            std::printf(
                "warp_spd revolute angle Hessian noise=%.0e map=%d: max|P - P_ref|/|M|_F "
                "mean / max: warp Jacobi %.3e / %.3e (at most %d sweeps), Eigen make_spd "
                "%.3e / %.3e, fixed-bank %.3e / %.3e\n",
                noise,
                map,
                sum_w / count,
                max_w,
                max_sweeps,
                sum_e / count,
                max_e,
                sum_f / count,
                max_f);
            const double replaced_mean = std::min(sum_e, sum_f) / count;
            const double replaced_max  = std::min(max_e, max_f);
            if(extended)
            {
                CHECK(sum_w / count <= 1.5 * replaced_mean + 1e-18);
                CHECK(max_w <= 3.0 * replaced_max + 1e-17);
            }
        }
    }
}

namespace
{
// The earlier warp Jacobi (library ilogb / scalbn, branch per entry in
// the update), kept here to check that the current warp_make_spd is bitwise
// the same iteration.
namespace committed_warp_spd
{
    UIPC_DEVICE inline void jacobi_rotation(
        Float app, Float apq, Float aqq, Float& c, Float& s, Float& t)
    {
        const int   e = ilogb(fmax(fabs(aqq - app), fabs(2 * apq)));
        const Float d = scalbn(aqq - app, -e);
        const Float w = scalbn(2 * apq, -e);
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

    // Returns the number of sweeps run (24 when the tolerance was not reached).
    template <int N>
    UIPC_DEVICE int warp_make_spd_committed(
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
                        if(apq != 0 && !(sweep >= SkipFrom && negligible(app, apq, aqq)))
                            jacobi_rotation(app, apq, aqq, c, s, t);
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
                        if(i == j)
                            A2[e] = A[e] + rot[3 * i + 2];
                        else if(pi == j)
                            A2[e] = 0;  // the annihilated entry
                        else
                        {
                            const Float x0 = ci * A[i + j * N] + si * A[pi + j * N];
                            const Float x1 = ci * A[i + pj * N] + si * A[pi + pj * N];
                            A2[e] = cj * x0 + sj * x1;
                        }
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
                    const Float value = subtract ? P[e] - scalbn(p, exponent) :
                                                   scalbn(p, exponent);
                    P[e]              = value;
                    P[j + i * N]      = value;
                }
            }
        }
        __syncwarp();
        return sweeps;
    }
}  // namespace committed_warp_spd

template <int N>
__global__ void warp_spd_committed_kernel(const Float* input, Float* output, int* sweeps, int count)
{
    const int I    = blockIdx.x;
    const int lane = threadIdx.x;
    if(I >= count)
        return;
    __shared__ Float A[N * N], A2[N * N], V[N * N], V2[N * N], P[N * N];
    __shared__ Float rot[warp_spd_rot_size<N>];
    __shared__ int   part[N];
    for(int e = lane; e < N * N; e += 32)
        A[e] = input[I * N * N + e];
    __syncwarp();
    const int s =
        committed_warp_spd::warp_make_spd_committed<N>(A, A2, V, V2, rot, part, P, lane);
    for(int e = lane; e < N * N; e += 32)
        output[I * N * N + e] = P[e];
    if(lane == 0)
        sweeps[I] = s;
}

// Every projection and sweep count of warp_make_spd equals the committed
// iteration's bit for bit.
template <int N>
void check_bitwise_committed(const std::vector<Float>& input, int count, const char* what)
{
    std::vector<int> sweeps, committed_sweeps;
    const auto       output = run_warp_spd<N>(input, count, sweeps);
    const auto       committed =
        run_warp_kernel<N>(warp_spd_committed_kernel<N>, input, count, committed_sweeps);
    int differing_entries = 0, differing_sweeps = 0;
    for(size_t k = 0; k < output.size(); ++k)
        differing_entries += std::memcmp(&output[k], &committed[k], sizeof(Float)) != 0;
    for(int I = 0; I < count; ++I)
        differing_sweeps += sweeps[I] != committed_sweeps[I];
    INFO(what << " (N = " << N << "): " << differing_entries << " differing entries, "
              << differing_sweeps << " differing sweep counts");
    CHECK(differing_entries == 0);
    CHECK(differing_sweeps == 0);
}
}  // namespace

TEST_CASE("warp Jacobi PSD projection is bitwise the committed iteration", "[warp_spd]")
{
    for(Float amplitude : {1e-6, 1.0, 1e6})
    {
        check_bitwise_committed<12>(warp_spd_test_inputs<12>(256, amplitude), 256, "test classes");
        check_bitwise_committed<9>(warp_spd_test_inputs<9>(256, amplitude), 256, "test classes");
    }

    // The library ilogb / scalbn path of the rotation: a random block that
    // keeps the sweeps going beside a block of equal diagonal entries coupled
    // far below them (|e| > 216) or by subnormal entries.
    std::mt19937_64                       gen(0x7a5d4444ULL);
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    for(Float tiny : {1e-70, 1e-200, 1e-310})
    {
        constexpr int      count = 64;
        std::vector<Float> input(count * 144);
        for(int I = 0; I < count; ++I)
        {
            Matrix12x12 M = Matrix12x12::Zero();
            for(int i = 0; i < 6; ++i)
                for(int j = 0; j <= i; ++j)
                    M(i, j) = M(j, i) = u(gen);
            for(int i = 6; i < 12; ++i)
            {
                M(i, i) = 0.5;
                for(int j = 6; j < i; ++j)
                    M(i, j) = M(j, i) = tiny * u(gen);
            }
            for(int e = 0; e < 144; ++e)
                input[I * 144 + e] = M.data()[e];
        }
        check_bitwise_committed<12>(input, count, "tiny couplings");
    }

    // revolute joint angle Hessians (both generated maps), as in the
    // convergence case above
    std::normal_distribution<Float> nd;
    auto                            frame = [&](Float noise)
    {
        using Vec3 = Eigen::Vector<Float, 3>;
        Vec3 axis  = Vec3(nd(gen), nd(gen), nd(gen)).normalized();
        Vec3 b     = axis.cross(Vec3(nd(gen), nd(gen), nd(gen))).normalized();
        Vec3 n     = axis.cross(b);
        auto R = Eigen::AngleAxis<Float>(3 * u(gen), axis).toRotationMatrix();
        auto G = Eigen::Quaternion<Float>(nd(gen), nd(gen), nd(gen), nd(gen))
                     .normalized()
                     .toRotationMatrix();
        Vector12 f;
        f << G * b, G * n, G * R * b, G * R * n;
        for(int k = 0; k < 12; ++k)
            f(k) += noise * nd(gen);
        return f;
    };
    for(Float noise : {0.0, 1e-6})
    {
        for(int map = 0; map < 2; ++map)
        {
            constexpr int         count = 256;
            std::vector<Vector12> F(count), F_t(count);
            std::vector<Float>    scale(count);
            for(int I = 0; I < count; ++I)
            {
                F[I]     = frame(noise);
                F_t[I]   = frame(noise);
                scale[I] = (I % 2 ? 1.0 : -1.0) * std::pow(10.0, 3 * u(gen));
            }
            Vector12 *d_F = nullptr, *d_Ft = nullptr;
            Float *   d_scale = nullptr, *d_H = nullptr;
            CUDA_TOOL_CHECK(cudaMalloc(&d_F, count * sizeof(Vector12)));
            CUDA_TOOL_CHECK(cudaMalloc(&d_Ft, count * sizeof(Vector12)));
            CUDA_TOOL_CHECK(cudaMalloc(&d_scale, count * sizeof(Float)));
            CUDA_TOOL_CHECK(cudaMalloc(&d_H, count * 144 * sizeof(Float)));
            CUDA_TOOL_CHECK(cudaMemcpy(d_F, F.data(), count * sizeof(Vector12), cudaMemcpyHostToDevice));
            CUDA_TOOL_CHECK(cudaMemcpy(d_Ft, F_t.data(), count * sizeof(Vector12), cudaMemcpyHostToDevice));
            CUDA_TOOL_CHECK(cudaMemcpy(d_scale, scale.data(), count * sizeof(Float), cudaMemcpyHostToDevice));
            revolute_angle_hessian_kernel<<<(count + 31) / 32, 32>>>(
                d_F, d_Ft, d_scale, d_H, count, map);
            CUDA_TOOL_CHECK(cudaGetLastError());
            std::vector<Float> input(count * 144);
            CUDA_TOOL_CHECK(cudaMemcpy(input.data(), d_H, input.size() * sizeof(Float), cudaMemcpyDeviceToHost));
            cudaFree(d_F);
            cudaFree(d_Ft);
            cudaFree(d_scale);
            cudaFree(d_H);
            check_bitwise_committed<12>(input, count, "revolute angle Hessians");
        }
    }
}

TEST_CASE("lane-parallel J^T H J matches the generated joint map", "[warp_spd]")
{
    std::mt19937_64                       gen(0x7a5d1111ULL);
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    for(int trial = 0; trial < 48; ++trial)
    {
        const int   map = trial % 3;
        Matrix12x12 H;
        for(int i = 0; i < 12; ++i)
            for(int j = 0; j <= i; ++j)
                H(i, j) = H(j, i) = u(gen);
        Vector6 bases[4];
        for(auto& b : bases)
            for(int i = 0; i < 6; ++i)
                b(i) = u(gen);

        Float *  d_h = nullptr, *d_a = nullptr, *d_b = nullptr;
        Vector6* d_bases = nullptr;
        CUDA_TOOL_CHECK(cudaMalloc(&d_h, 144 * sizeof(Float)));
        CUDA_TOOL_CHECK(cudaMalloc(&d_a, 576 * sizeof(Float)));
        CUDA_TOOL_CHECK(cudaMalloc(&d_b, 576 * sizeof(Float)));
        CUDA_TOOL_CHECK(cudaMalloc(&d_bases, sizeof(bases)));
        CUDA_TOOL_CHECK(cudaMemcpy(d_h, H.data(), 144 * sizeof(Float), cudaMemcpyHostToDevice));
        CUDA_TOOL_CHECK(cudaMemcpy(d_bases, bases, sizeof(bases), cudaMemcpyHostToDevice));
        if(map == 2)
            basis_jt_h_j_test_kernel<true><<<1, 32>>>(d_h, d_bases, d_a);
        else
            basis_jt_h_j_test_kernel<false><<<1, 32>>>(d_h, d_bases, d_a);
        sym_jt_h_j_test_kernel<<<1, 1>>>(d_h, d_bases, d_b, map);
        CUDA_TOOL_CHECK(cudaGetLastError());
        std::vector<Float> a(576), b(576);
        CUDA_TOOL_CHECK(cudaMemcpy(a.data(), d_a, 576 * sizeof(Float), cudaMemcpyDeviceToHost));
        CUDA_TOOL_CHECK(cudaMemcpy(b.data(), d_b, 576 * sizeof(Float), cudaMemcpyDeviceToHost));
        cudaFree(d_h);
        cudaFree(d_a);
        cudaFree(d_b);
        cudaFree(d_bases);
        Float scale = 0, diff = 0;
        for(int e = 0; e < 576; ++e)
        {
            scale = std::max(scale, std::abs(b[e]));
            diff  = std::max(diff, std::abs(a[e] - b[e]));
        }
        INFO("map " << map << ": max diff " << diff << " of max " << scale);
        CHECK(diff <= 1e-13 * scale);
    }
}

namespace
{
// map 0: prismatic joint (J01^T H J01 + ddE23), 1: driving prismatic / limit
__global__ void prismatic_lane_kernel(const Float* H01, const Vector3* v, Float kappa, int map, Float* out)
{
    const int lane = threadIdx.x;
    for(int e = lane; e < 576; e += 32)
    {
        const int a = e % 24, b = e / 24;
        out[e] =
            map == 0 ?
                prismatic_jt_h_j_entry<1>(H01, v[0], v[1], v[2], v[3], a, b)
                    + prismatic_e23_hessian_entry(kappa, v[4], v[5], v[6], v[7], a, b) :
                prismatic_jt_h_j_entry<-1>(H01, v[0], v[1], v[2], v[3], a, b);
    }
}

__global__ void prismatic_sym_kernel(const Float* H01, const Vector3* v, Float kappa, int map, Float* out)
{
    Matrix9x9                    H = Eigen::Map<const Matrix9x9>(H01);
    Eigen::Matrix<Float, 24, 24> R;
    if(map == 0)
    {
        namespace PJ = sym::affine_body_prismatic_joint;
        Eigen::Matrix<Float, 24, 24> E;
        Vector12                     q = Vector12::Zero();
        PJ::J01T_H01_J01<Float>(R, H, v[0], v[1], v[2], v[3]);
        PJ::ddE23ddQ<Float>(E, kappa, v[4], v[5], q, v[6], v[7], q);
        R += E;
    }
    else
    {
        sym::affine_body_driving_prismatic_joint::J01T_H01_J01<Float>(
            R, H, v[0], v[1], v[2], v[3]);
    }
    for(int e = 0; e < 576; ++e)
        out[e] = R(e % 24, e / 24);
}
}  // namespace

TEST_CASE("lane-parallel prismatic J01^T H J01 matches the generated maps", "[warp_spd]")
{
    std::mt19937_64                       gen(0x7a5d2222ULL);
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    for(int trial = 0; trial < 32; ++trial)
    {
        const int map = trial % 2;
        Matrix9x9 H;
        for(int i = 0; i < 9; ++i)
            for(int j = 0; j <= i; ++j)
                H(i, j) = H(j, i) = u(gen);
        Vector3 v[8];
        for(auto& x : v)
            for(int i = 0; i < 3; ++i)
                x(i) = u(gen);
        const Float kappa = 1e3 * (1.5 + u(gen));

        Float *  d_h = nullptr, *d_a = nullptr, *d_b = nullptr;
        Vector3* d_v = nullptr;
        CUDA_TOOL_CHECK(cudaMalloc(&d_h, 81 * sizeof(Float)));
        CUDA_TOOL_CHECK(cudaMalloc(&d_a, 576 * sizeof(Float)));
        CUDA_TOOL_CHECK(cudaMalloc(&d_b, 576 * sizeof(Float)));
        CUDA_TOOL_CHECK(cudaMalloc(&d_v, sizeof(v)));
        CUDA_TOOL_CHECK(cudaMemcpy(d_h, H.data(), 81 * sizeof(Float), cudaMemcpyHostToDevice));
        CUDA_TOOL_CHECK(cudaMemcpy(d_v, v, sizeof(v), cudaMemcpyHostToDevice));
        prismatic_lane_kernel<<<1, 32>>>(d_h, d_v, kappa, map, d_a);
        prismatic_sym_kernel<<<1, 1>>>(d_h, d_v, kappa, map, d_b);
        CUDA_TOOL_CHECK(cudaGetLastError());
        std::vector<Float> a(576), b(576);
        CUDA_TOOL_CHECK(cudaMemcpy(a.data(), d_a, 576 * sizeof(Float), cudaMemcpyDeviceToHost));
        CUDA_TOOL_CHECK(cudaMemcpy(b.data(), d_b, 576 * sizeof(Float), cudaMemcpyDeviceToHost));
        cudaFree(d_h);
        cudaFree(d_a);
        cudaFree(d_b);
        cudaFree(d_v);
        Float scale = 0, diff = 0;
        for(int e = 0; e < 576; ++e)
        {
            scale = std::max(scale, std::abs(b[e]));
            diff  = std::max(diff, std::abs(a[e] - b[e]));
        }
        INFO("map " << map << ": max diff " << diff << " of max " << scale);
        CHECK(diff <= 1e-13 * scale);
    }
}
