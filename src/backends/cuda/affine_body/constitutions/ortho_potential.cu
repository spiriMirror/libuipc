/*
 * NVIDIA contributions: shared-workspace material PSD projection and
 * gradient-only launch selection; launches on the provided assembly streams
 * with one-warp launch shapes.
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

#include <utils/material_gradient_hessian_launch.h>
#include <affine_body/affine_body_constitution.h>
#include <affine_body/constitutions/ortho_potential_function.h>
#include <utils/fixed_bank_soa_evd.h>


namespace uipc::backend::cuda
{
namespace
{
    namespace AOP = sym::abd_ortho_potential;

    __global__ void ortho_potential_compute_energy_kernel(
        cuda_tool::BufferView<Float>     shape_energies,
        cuda_tool::CBufferView<Vector12> qs,
        cuda_tool::CBufferView<Float>    kappas,
        cuda_tool::CBufferView<Float>    volumes,
        Float                            dt,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& q      = qs(i);
        auto& volume = volumes(i);
        auto  kappa  = kappas(i);
        Float Vdt2   = volume * dt * dt;

        Float E;
        AOP::E(E, kappa, q);

        shape_energies(i) = E * Vdt2;
    }

    template <bool GradientOnly>
    __global__ void ortho_potential_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Vector12>   qs,
        cuda_tool::CBufferView<Float>      volumes,
        cuda_tool::BufferView<Vector12>    gradients,
        cuda_tool::BufferView<Matrix12x12> body_hessian,
        cuda_tool::CBufferView<Float>      kappas,
        Float                              dt,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector12 G = Vector12::Zero();

        const auto& q      = qs(i);
        Float       kappa  = kappas(i);
        const auto& volume = volumes(i);

        Float Vdt2 = volume * dt * dt;

        Vector9 G9;
        AOP::dEdq(G9, kappa, q);
        G.segment<9>(3) = G9 * Vdt2;
        gradients(i)    = G;

        if constexpr(!GradientOnly)
        {
            // one slot of the pitch per thread of the block
            constexpr int SharedLanePitch = 16;
            UIPC_KERNEL_ASSERT(blockDim.x <= SharedLanePitch,
                               "ortho potential Hessian: %d threads per block, at most %d",
                               (int)blockDim.x,
                               SharedLanePitch);
            __shared__ Float shared_h[9 * 9 * SharedLanePitch];

            FixedBankSoAMap<9, SharedLanePitch> H9x9(shared_h + threadIdx.x);
            AOP::ddEddq(H9x9, kappa, q);

            Matrix12x12 H              = Matrix12x12::Zero();
            auto        H9x9_projected = H.block<9, 9>(3, 3);
            make_spd_fixed_bank_shared_upper_fma<9>(H9x9, H9x9_projected);
            H9x9_projected *= Vdt2;
            body_hessian(i) = H;
        }
    }
}  // namespace

void launch_ortho_potential_gradient_hessian(const OrthoPotentialGradientHessianLaunchInfo& info)
{
    using namespace cuda_tool;
    auto N             = info.qs.size();
    auto gradient_only = info.gradient_only;

    namespace AOP = sym::abd_ortho_potential;

    int n = (int)N;
    if(n <= 0)
        return;

    if(gradient_only)
    {
        auto k = ortho_potential_compute_gradient_hessian_kernel<true>;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream>>>(
            info.qs, info.volumes, info.gradients, info.hessians, info.kappas, info.dt, n);
    }
    else
    {
        constexpr int BlockSize = 16;
        auto k = ortho_potential_compute_gradient_hessian_kernel<false>;
        k<<<(n + BlockSize - 1) / BlockSize, BlockSize, 0, info.stream>>>(
            info.qs, info.volumes, info.gradients, info.hessians, info.kappas, info.dt, n);
    }
}

class OrthoPotential final : public AffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 1ull;

    using AffineBodyConstitution::AffineBodyConstitution;

    vector<Float> h_kappas;

    cuda_tool::DeviceBuffer<Float> kappas;

    virtual void do_build(AffineBodyConstitution::BuildInfo& info) override {}

    U64 get_uid() const override { return ConstitutionUID; }

    void do_init(AffineBodyDynamics::FilteredInfo& info) override
    {
        using ForEachInfo = AffineBodyDynamics::ForEachInfo;

        // find out constitution coefficients
        h_kappas.resize(info.body_count());
        auto geo_slots = world().scene().geometries();

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.instances().find<Float>("kappa")->view(); },
            [&](const ForEachInfo& I, Float kappa)
            {
                auto bodyI      = I.global_index();
                h_kappas[bodyI] = kappa;
            });

        auto async_copy = []<typename T>(span<T> src, cuda_tool::DeviceBuffer<T>& dst)
        {
            cuda_tool::BufferLaunch().resize<T>(dst, src.size());
            cuda_tool::BufferLaunch().copy<T>(dst.view(), src.data());
        };

        async_copy(span{h_kappas}, kappas);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;

        auto body_count = info.qs().size();

        namespace AOP = sym::abd_ortho_potential;

        auto k = ortho_potential_compute_energy_kernel;
        int  n = (int)body_count;
        if(n > 0)
            k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, nullptr>>>(
                info.energies(), info.qs(), kappas.cview(), info.volumes(), info.dt(), n);
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        launch_ortho_potential_gradient_hessian(
            OrthoPotentialGradientHessianLaunchInfo{info.qs(),
                                                    info.volumes(),
                                                    info.gradients(),
                                                    info.hessians(),
                                                    kappas.cview(),
                                                    info.dt(),
                                                    info.gradient_only(),
                                                    info.stream()});
    }
};

REGISTER_SIM_SYSTEM(OrthoPotential);
}  // namespace uipc::backend::cuda
