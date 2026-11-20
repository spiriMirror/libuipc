/*
 * NVIDIA contributions: shared-workspace shell PSD projection, energy-launch
 * specialization and gradient-only launch selection; launches on the provided
 * stream for the prelaunched FEM assembly.
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
#include <finite_element/codim_2d_constitution.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_energy.h>
#include <finite_element/constitutions/neo_hookean_shell_2d_function.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Dense>
#include <utils/codim_thickness.h>
#include <utils/fixed_bank_soa_evd.h>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace NH = sym::neo_hookean_shell_2d;

    constexpr SizeT StencilSize     = 3;
    constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    __global__ void NeoHookeanShell2D_do_init_kernel(cuda_tool::CBufferView<Vector3i> prims,
                                                     cuda_tool::CBufferView<Vector3> x_bars,
                                                     cuda_tool::BufferView<Matrix2x2> inv_B_mats,
                                                     int n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector9  X_bar;
        Vector3i idx = prims(I);
        for(int i = 0; i < 3; ++i)
            X_bar.segment<3>(3 * i) = x_bars(idx(i));
        Matrix2x2 B;
        NH::A(B, X_bar);

        inv_B_mats(I) = cuda_tool::eigen::inverse(B);
    }

    __global__ void NeoHookeanShell2D_do_compute_energy_kernel(
        cuda_tool::CBufferView<Float>     lambdas,
        cuda_tool::CBufferView<Float>     mus,
        cuda_tool::CBufferView<Float>     rest_areas,
        cuda_tool::CBufferView<Float>     thicknesses,
        cuda_tool::BufferView<Float>      energies,
        cuda_tool::CBufferView<Vector3i>  indices,
        cuda_tool::CBufferView<Vector3>   xs,
        cuda_tool::CBufferView<Matrix2x2> IBs,
        Float                             dt,
        int                               n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector9          X;
        Vector3i         idx = indices(I);
        const Matrix2x2& IB  = IBs(I);
        for(int i = 0; i < 3; ++i)
            X.segment<3>(3 * i) = xs(idx(i));

        Float lambda = lambdas(I);
        Float mu     = mus(I);

        Float rest_area = rest_areas(I);
        Float thickness = triangle_thickness(
            thicknesses(idx(0)), thicknesses(idx(1)), thicknesses(idx(2)));
        Float E;
        NH::E(E, lambda, mu, X, IB);

        // thickness is one-sided so we multiply by 2
        Float Vdt2  = rest_area * 2 * thickness * dt * dt;
        energies(I) = E * Vdt2;
    }

    template <bool GradientOnly>
    __global__ void NeoHookeanShell2D_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Float>          lambdas,
        cuda_tool::CBufferView<Float>          mus,
        cuda_tool::CBufferView<Vector3i>       indices,
        cuda_tool::CBufferView<Vector3>        xs,
        cuda_tool::CBufferView<Matrix2x2>      IBs,
        cuda_tool::CBufferView<Float>          thicknesses,
        cuda_tool::DoubletVectorView<Float, 3> G3s,
        cuda_tool::TripletMatrixView<Float, 3> H3x3s,
        cuda_tool::CBufferView<Float>          rest_areas,
        Float                                  dt,
        SizeT                                  half_hessian_size,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector9  X;
        Vector3i idx = indices(I);
        for(int i = 0; i < 3; ++i)
            X.segment<3>(3 * i) = xs(idx(i));

        Matrix2x2 IB = IBs(I);

        Float lambda    = lambdas(I);
        Float mu        = mus(I);
        Float rest_area = rest_areas(I);
        Float thickness = triangle_thickness(
            thicknesses(idx(0)), thicknesses(idx(1)), thicknesses(idx(2)));

        // thickness is one-sided so we multiply by 2
        Float Vdt2 = rest_area * 2 * thickness * dt * dt;

        Vector9 G;
        NH::dEdX(G, lambda, mu, X, IB);
        G *= Vdt2;
        DoubletVectorAssembler DVA{G3s};
        DVA.segment<StencilSize>(I * StencilSize).write(idx, G);

        if constexpr(!GradientOnly)
        {
            constexpr int    SharedLanePitch = 32;
            __shared__ Float shared_h[9 * 9 * SharedLanePitch];

            FixedBankSoAMap<9, SharedLanePitch> H(shared_h + threadIdx.x);
            NH::ddEddX(H, lambda, mu, X, IB);

            Vector9 eigen_values;
            selfadjoint_evd_fixed_bank_shared<9>(H, eigen_values);
            TripletMatrixAssembler TMA{H3x3s};
            TMA.half_block<StencilSize>(I * half_hessian_size)
                .write_psd_from_eigendecomposition(idx, H, eigen_values, Vdt2);
        }
    }
}  // namespace

void launch_neo_hookean_shell_2d_energy(const NeoHookeanShell2DEnergyLaunchInfo& info)
{
    const int n = static_cast<int>(info.indices.size());
    if(n == 0)
        return;

    constexpr int BlockSize = 64;
    NeoHookeanShell2D_do_compute_energy_kernel<<<(n + BlockSize - 1) / BlockSize, BlockSize, 0, nullptr>>>(
        info.lambdas,
        info.mus,
        info.rest_areas,
        info.thicknesses,
        info.energies,
        info.indices,
        info.positions,
        info.inverse_rest_shape_matrices,
        info.dt,
        n);
}

void launch_neo_hookean_shell_gradient_hessian(const NeoHookeanShellGradientHessianLaunchInfo& info)
{
    int n = (int)info.indices.size();
    if(n <= 0)
        return;

    if(info.gradient_only)
    {
        auto k = NeoHookeanShell2D_do_compute_gradient_hessian_kernel<true>;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream>>>(
            info.lambdas,
            info.mus,
            info.indices,
            info.positions,
            info.inverse_rest_shape_matrices,
            info.thicknesses,
            info.gradients,
            info.hessians,
            info.rest_areas,
            info.dt,
            HalfHessianSize,
            n);
    }
    else
    {
        constexpr int BlockSize = 32;
        auto k = NeoHookeanShell2D_do_compute_gradient_hessian_kernel<false>;
        k<<<(n + BlockSize - 1) / BlockSize, BlockSize, 0, info.stream>>>(
            info.lambdas,
            info.mus,
            info.indices,
            info.positions,
            info.inverse_rest_shape_matrices,
            info.thicknesses,
            info.gradients,
            info.hessians,
            info.rest_areas,
            info.dt,
            HalfHessianSize,
            n);
    }
}

class NeoHookeanShell2D final : public Codim2DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 11;
    static constexpr SizeT StencilSize     = 3;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using Codim2DConstitution::Codim2DConstitution;

    vector<Float> h_lambdas;
    vector<Float> h_mus;

    cuda_tool::DeviceBuffer<Float>     lambdas;
    cuda_tool::DeviceBuffer<Float>     mus;
    cuda_tool::DeviceBuffer<Matrix2x2> inv_B_matrices;

    SimSystemSlot<FiniteElementMethod> fem;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override
    {
        fem = require<FiniteElementMethod>();
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_mus.resize(N);
        h_lambdas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto lambda = sc.triangles().find<Float>("lambda");
                auto mu     = sc.triangles().find<Float>("mu");


                return zip(lambda->view(), mu->view());
            },
            [&](const ForEachInfo& I, auto lambda_mu)
            {
                auto vI = I.global_index();

                auto&& [lambda, mu] = lambda_mu;

                h_lambdas[vI] = lambda;
                h_mus[vI]     = mu;
            });

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());

        mus.resize(N);
        mus.view().copy_from(h_mus.data());

        auto& cinfo       = info.constitution_info();
        auto  prim_offset = cinfo.primitive_offset;
        auto  prim_count  = cinfo.primitive_count;

        auto prims  = fem->codim_2ds().subview(prim_offset, prim_count);
        auto x_bars = fem->x_bars();

        inv_B_matrices.resize(N);

        // Precompute inverse of rest shape matrix for each triangle
        auto k = NeoHookeanShell2D_do_init_kernel;
        int  n = (int)N;
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                prims, x_bars, inv_B_matrices.view(), n);
        }
    }

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(mus.size());
        info.gradient_count(mus.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(mus.size() * HalfHessianSize);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        launch_neo_hookean_shell_2d_energy(NeoHookeanShell2DEnergyLaunchInfo{
            .lambdas                     = lambdas.cview(),
            .mus                         = mus.cview(),
            .rest_areas                  = info.rest_areas(),
            .thicknesses                 = info.thicknesses(),
            .energies                    = info.energies(),
            .indices                     = info.indices(),
            .positions                   = info.xs(),
            .inverse_rest_shape_matrices = inv_B_matrices.cview(),
            .dt                          = info.dt()});
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        launch_neo_hookean_shell_gradient_hessian(
            NeoHookeanShellGradientHessianLaunchInfo{lambdas.cview(),
                                                     mus.cview(),
                                                     info.indices(),
                                                     info.xs(),
                                                     inv_B_matrices.cview(),
                                                     info.thicknesses(),
                                                     info.gradients(),
                                                     info.hessians(),
                                                     info.rest_areas(),
                                                     info.dt(),
                                                     info.gradient_only(),
                                                     info.stream()});
    }
};

REGISTER_SIM_SYSTEM(NeoHookeanShell2D);
}  // namespace uipc::backend::cuda
