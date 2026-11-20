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

// The ABD kinetic, joint and constraint producers of the assembly and their
// line-search energies launch one-warp blocks (cuda_tool::spread_block_dim)
// instead of the occupancy-maximizing block. This case runs the fixed
// joint's energy, gradient and PSD-projected Hessian and the revolute
// joint's energy, as their kernels compute them per thread, both ways on
// random joints and requires the same bits from every item.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <affine_body/constitutions/affine_body_fixed_joint_function.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>
#include <utils/make_spd.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
namespace FJ = sym::affine_body_fixed_joint;
namespace RJ = sym::affine_body_revolute_joint;

using Vector24    = Vector<Float, 24>;
using Matrix24x24 = Matrix<Float, 24, 24>;

// Joint I joins bodies 2I and 2I + 1; frames[4I .. 4I + 3] hold its rest
// center, tangent, normal and binormal (both bodies' halves). The lines
// mirror affine_body_fixed_joint_compute_{energy,gradient_hessian}_kernel
// and affine_body_revolute_joint_compute_energy_kernel.
__global__ void abd_launch_shape_kernel(const Vector12* qs,
                                        const Vector6*  frames,
                                        const Float*    kappas,
                                        Float*          energies,
                                        Vector24*       gradients,
                                        Matrix24x24*    hessians,
                                        int             n)
{
    int I = blockIdx.x * blockDim.x + threadIdx.x;
    if(I >= n)
        return;
    const Vector12& qi     = qs[2 * I];
    const Vector12& qj     = qs[2 * I + 1];
    const Vector6&  rest_c = frames[4 * I];
    const Vector6&  rest_t = frames[4 * I + 1];
    const Vector6&  rest_n = frames[4 * I + 2];
    const Vector6&  rest_b = frames[4 * I + 3];
    Float           kappa  = kappas[I];

    Vector9 Fr_val;
    FJ::Fr<Float>(Fr_val,
                  rest_t.segment<3>(0),
                  rest_n.segment<3>(0),
                  rest_b.segment<3>(0),
                  qi,
                  rest_t.segment<3>(3),
                  rest_n.segment<3>(3),
                  rest_b.segment<3>(3),
                  qj);
    Float Er_val;
    FJ::Er(Er_val, kappa, Fr_val);
    Vector3 Ft_val;
    FJ::Ft<Float>(Ft_val, rest_c.segment<3>(0), qi, rest_c.segment<3>(3), qj);
    Float Et_val;
    FJ::Et(Et_val, kappa, Ft_val);
    energies[2 * I] = Er_val + Et_val;

    Vector6 F;
    RJ::Faxis<Float>(F,
                     rest_t.segment<3>(0),
                     rest_n.segment<3>(0),
                     qi,
                     rest_t.segment<3>(3),
                     rest_n.segment<3>(3),
                     qj);
    Float E;
    RJ::Eaxis<Float>(E, kappa, F);
    energies[2 * I + 1] = E;

    Vector9 dEdFr_val;
    FJ::dEdFr(dEdFr_val, kappa, Fr_val);
    Vector24 JrT_Gr_val;
    FJ::JrT_Gr<Float>(JrT_Gr_val,
                      dEdFr_val,
                      rest_t.segment<3>(0),
                      rest_n.segment<3>(0),
                      rest_b.segment<3>(0),
                      qi,
                      rest_t.segment<3>(3),
                      rest_n.segment<3>(3),
                      rest_b.segment<3>(3),
                      qj);
    Vector3 dEdFt_val;
    FJ::dEdFt(dEdFt_val, kappa, Ft_val);
    Vector24 JtT_Gt_val;
    FJ::JtT_Gt<Float>(JtT_Gt_val, dEdFt_val, rest_c.segment<3>(0), rest_c.segment<3>(3));
    gradients[I] = JrT_Gr_val + JtT_Gt_val;

    Matrix9x9 ddEdFr_val;
    FJ::ddEdFr(ddEdFr_val, kappa, Fr_val);
    make_spd(ddEdFr_val);
    Matrix24x24 JrT_Hr_Jr_val;
    FJ::JrT_Hr_Jr<Float>(JrT_Hr_Jr_val,
                         ddEdFr_val,
                         rest_t.segment<3>(0),
                         rest_n.segment<3>(0),
                         rest_b.segment<3>(0),
                         qi,
                         rest_t.segment<3>(3),
                         rest_n.segment<3>(3),
                         rest_b.segment<3>(3),
                         qj);
    Matrix3x3 ddEdFt_val;
    FJ::ddEdFt(ddEdFt_val, kappa, Ft_val);
    make_spd(ddEdFt_val);
    Matrix24x24 JtT_Ht_Jt_val;
    FJ::JtT_Ht_Jt<Float>(
        JtT_Ht_Jt_val, ddEdFt_val, rest_c.segment<3>(0), rest_c.segment<3>(3));
    hessians[I] = JrT_Hr_Jr_val + JtT_Ht_Jt_val;
}

struct Outputs
{
    std::vector<Float>       energies;
    std::vector<Vector24>    gradients;
    std::vector<Matrix24x24> hessians;
};

Outputs run(int                          n,
            bool                         spread,
            const std::vector<Vector12>& h_qs,
            const std::vector<Vector6>&  h_frames,
            const std::vector<Float>&    h_kappas)
{
    cuda_tool::DeviceBuffer<Vector12>    qs;
    cuda_tool::DeviceBuffer<Vector6>     frames;
    cuda_tool::DeviceBuffer<Float>       kappas, energies;
    cuda_tool::DeviceBuffer<Vector24>    gradients;
    cuda_tool::DeviceBuffer<Matrix24x24> hessians;
    qs.copy_from(h_qs.data(), h_qs.size());
    frames.copy_from(h_frames.data(), h_frames.size());
    kappas.copy_from(h_kappas.data(), h_kappas.size());
    // NaN in every slot, so an item a launch shape misses cannot pass.
    const Float              nan = std::numeric_limits<Float>::quiet_NaN();
    std::vector<Float>       e(2 * n, nan);
    std::vector<Vector24>    g(n, Vector24::Constant(nan));
    std::vector<Matrix24x24> h(n, Matrix24x24::Constant(nan));
    energies.copy_from(e.data(), e.size());
    gradients.copy_from(g.data(), g.size());
    hessians.copy_from(h.data(), h.size());

    auto k = abd_launch_shape_kernel;
    if(spread)
        k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim>>>(
            qs.data(),
            frames.data(),
            kappas.data(),
            energies.data(),
            gradients.data(),
            hessians.data(),
            n);
    else
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            qs.data(),
            frames.data(),
            kappas.data(),
            energies.data(),
            gradients.data(),
            hessians.data(),
            n);
    CUDA_TOOL_CHECK(cudaGetLastError());

    Outputs out;
    energies.copy_to(out.energies);
    gradients.copy_to(out.gradients);
    hessians.copy_to(out.hessians);
    return out;
}

void check_launch_shape(int n, std::uint64_t seed)
{
    std::mt19937_64                       rng{seed};
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    auto rand3 = [&](Float s) { return Vector3{u(rng), u(rng), u(rng)} * s; };

    // Two bodies near a shared joint frame: positions within a meter,
    // deformation gradients near a rotation, rest frames near orthonormal.
    std::vector<Vector12> qs;
    std::vector<Vector6>  frames;
    std::vector<Float>    kappas;
    for(int i = 0; i < n; ++i)
    {
        for(int b = 0; b < 2; ++b)
        {
            Vector12 q;
            q.segment<3>(0) = rand3(1.0);
            Matrix3x3 A     = Matrix3x3::Identity();
            for(int r = 0; r < 3; ++r)
                A.row(r) += 0.05 * rand3(1.0).transpose();
            for(int r = 0; r < 3; ++r)
                q.segment<3>(3 + 3 * r) = A.row(r).transpose();
            qs.push_back(q);
        }
        Vector6 c, t, nn, bb;
        c << rand3(0.5), rand3(0.5);
        t << Vector3{1.0, 0.0, 0.0} + rand3(0.02), Vector3{1.0, 0.0, 0.0} + rand3(0.02);
        nn << Vector3{0.0, 1.0, 0.0} + rand3(0.02), Vector3{0.0, 1.0, 0.0} + rand3(0.02);
        bb << Vector3{0.0, 0.0, 1.0} + rand3(0.02), Vector3{0.0, 0.0, 1.0} + rand3(0.02);
        frames.push_back(c);
        frames.push_back(t);
        frames.push_back(nn);
        frames.push_back(bb);
        kappas.push_back(std::pow(10.0, 3.0 + 2.0 * (0.5 + 0.5 * u(rng))));
    }

    Outputs a = run(n, false, qs, frames, kappas);
    Outputs b = run(n, true, qs, frames, kappas);
    REQUIRE(a.energies.size() == SizeT(2 * n));
    REQUIRE(b.energies.size() == SizeT(2 * n));
    REQUIRE(std::memcmp(a.energies.data(), b.energies.data(), sizeof(Float) * 2 * n) == 0);
    REQUIRE(std::memcmp(a.gradients.data(), b.gradients.data(), sizeof(Vector24) * n) == 0);
    REQUIRE(std::memcmp(a.hessians.data(), b.hessians.data(), sizeof(Matrix24x24) * n) == 0);

    SizeT finite = 0;
    for(int i = 0; i < n; ++i)
        finite += std::isfinite(b.energies[2 * i])
                  && std::isfinite(b.energies[2 * i + 1])
                  && b.gradients[i].allFinite() && b.hessians[i].allFinite();
    REQUIRE(finite == SizeT(n));
}
}  // namespace

TEST_CASE("ABD joint producers give the same bits with one-warp blocks", "[cuda][abd][launch]")
{
    REQUIRE(cuda_tool::spread_block_dim == 32);
    int seed = 0;
    for(int n : {1, 31, 33, 1000})
        check_launch_shape(n, 303 + seed++);
}
