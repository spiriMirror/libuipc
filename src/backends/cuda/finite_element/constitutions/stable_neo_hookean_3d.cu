/*
 * NVIDIA contributions: assembly and energy launches on the provided streams
 * and direct gradient accumulation for the in-place prelaunch; Hessian
 * projection products shared per left vertex and the grid-capped producer
 * launch, as one round of the narrowest wide block that covers the elements or
 * as a grid-stride loop.
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

#include <algorithm>
#include <finite_element/fem_3d_constitution.h>
#include <finite_element/constitutions/stable_neo_hookean_3d_function.h>
#include <finite_element/fem_utils.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Dense>
#include <utils/matrix_assembler.h>

namespace uipc::backend::cuda
{
namespace
{
    // Stiff-GIPC SNK1 (energy, gradient and the analytically SPD-projected
    // Hessian) replaces the SymEigen-generated SNH + generic make_spd EVD
    namespace SNH = snk1;

    constexpr SizeT StencilSize     = 4;
    constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    __global__ void StableNeoHookean3D_do_compute_energy_kernel(
        cuda_tool::CBufferView<Float>     mus,
        cuda_tool::CBufferView<Float>     lambdas,
        cuda_tool::BufferView<Float>      energies,
        cuda_tool::CBufferView<Vector4i>  indices,
        cuda_tool::CBufferView<Vector3>   xs,
        cuda_tool::CBufferView<Matrix3x3> Dm_invs,
        cuda_tool::CBufferView<Float>     volumes,
        Float                             dt,
        int                               n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const Vector4i&  tet    = indices(I);
        const Matrix3x3& Dm_inv = Dm_invs(I);
        Float            mu     = mus(I);
        Float            lambda = lambdas(I);

        const Vector3& x0 = xs(tet(0));
        const Vector3& x1 = xs(tet(1));
        const Vector3& x2 = xs(tet(2));
        const Vector3& x3 = xs(tet(3));

        auto F = fem::F(x0, x1, x2, x3, Dm_inv);

        Float E;

        SNH::E(E, mu, lambda, F);
        E *= dt * dt * volumes(I);
        energies(I) = E;
    }

    // Gradient and Hessian of one tetrahedron. Direct: the gradient is added
    // to the global gradient (FEMLinearSubsystem::AssembleInfo::
    // direct_gradients(), skipping fixed vertices as the doublet addition
    // does) instead of written as doublets; the arithmetic is the same.
    template <bool Direct>
    __device__ __forceinline__ void StableNeoHookean3D_gradient_hessian(
        int                                    I,
        cuda_tool::CBufferView<Float>          mus,
        cuda_tool::CBufferView<Float>          lambdas,
        cuda_tool::CBufferView<Vector4i>       indices,
        cuda_tool::CBufferView<Vector3>        xs,
        cuda_tool::CBufferView<Matrix3x3>      Dm_invs,
        cuda_tool::DoubletVectorView<Float, 3> G3s,
        cuda_tool::DenseVectorView<Float>      direct_gradients,
        cuda_tool::CBufferView<IndexT>         is_fixed,
        cuda_tool::TripletMatrixView<Float, 3> H3x3s,
        cuda_tool::CBufferView<Float>          volumes,
        Float                                  dt,
        bool                                   gradient_only)
    {
        const Vector4i&  tet    = indices(I);
        const Matrix3x3& Dm_inv = Dm_invs(I);
        Float            mu     = mus(I);
        Float            lambda = lambdas(I);

        const Vector3& x0 = xs(tet(0));
        const Vector3& x1 = xs(tet(1));
        const Vector3& x2 = xs(tet(2));
        const Vector3& x3 = xs(tet(3));

        auto F = fem::F(x0, x1, x2, x3, Dm_inv);

        auto Vdt2 = volumes(I) * dt * dt;

        Matrix3x3 dEdF;
        SNH::dEdF(dEdF, mu, lambda, F);
        auto VecdEdF = flatten(dEdF);
        VecdEdF *= Vdt2;

        auto shape_gradients = fem::tetrahedron_shape_gradients(Dm_inv);

        if constexpr(Direct)
        {
#pragma unroll
            for(int i = 0; i < StencilSize; ++i)
            {
                Vector3 G = fem::project_F_gradient(shape_gradients.col(i), VecdEdF);
                if(!is_fixed(tet(i)))
                    direct_gradients.segment<3>(tet(i) * 3).atomic_add(G);
            }
        }
        else
        {
            DoubletVectorAssembler DVA{G3s};
#pragma unroll
            for(int i = 0; i < StencilSize; ++i)
            {
                Vector3 G = fem::project_F_gradient(shape_gradients.col(i), VecdEdF);
                DVA(I * StencilSize + i).write(tet(i), G);
            }
        }

        if(gradient_only)
            return;

        // analytically SPD-projected 9x9 energy Hessian (Stiff SNK1);
        // scaling by the positive Vdt2 commutes with the projection
        Matrix9x9 ddEddF;
        SNH::ddEddF_spd(ddEddF, mu, lambda, F);
        ddEddF *= Vdt2;

        // Block (i, j), i <= j, sits at offset i * (7 - i) / 2 + j and its left
        // vertex is the one with the smaller global index (ties keep the local
        // order), i.e. the lower rank below. Visiting the blocks by rank lets
        // each left vertex form its products shape_gradient(k) *
        // ddEddF(3k+a, 3l+b) once for all of its blocks; every block still
        // adds the same terms in the same order as
        // fem::project_F_hessian_block, so the result is bitwise unchanged.
        int rank[StencilSize];
#pragma unroll
        for(int v = 0; v < StencilSize; ++v)
        {
            rank[v] = 0;
#pragma unroll
            for(int u = 0; u < StencilSize; ++u)
            {
                if(u != v && (tet(u) < tet(v) || (tet(u) == tet(v) && u < v)))
                    ++rank[v];
            }
        }

        int     local[StencilSize];  // local vertex of each rank
        Vector3 G[StencilSize];      // its shape gradient
#pragma unroll
        for(int r = 0; r < StencilSize; ++r)
        {
            local[r] = 0;
            G[r]     = shape_gradients.col(0);
#pragma unroll
            for(int v = 1; v < StencilSize; ++v)
            {
                if(rank[v] == r)
                {
                    local[r] = v;
                    G[r]     = shape_gradients.col(v);
                }
            }
        }

#pragma unroll
        for(int r = 0; r < StencilSize; ++r)
        {
            Matrix3x3 H[StencilSize];  // blocks (r, s), s >= r
#pragma unroll
            for(int s = r; s < StencilSize; ++s)
                H[s].setZero();

#pragma unroll
            for(int a = 0; a < 3; ++a)
            {
#pragma unroll
                for(int b = 0; b < 3; ++b)
                {
#pragma unroll
                    for(int k = 0; k < 3; ++k)
                    {
#pragma unroll
                        for(int l = 0; l < 3; ++l)
                        {
                            Float product = G[r](k) * ddEddF(3 * k + a, 3 * l + b);
#pragma unroll
                            for(int s = r; s < StencilSize; ++s)
                                H[s](a, b) += product * G[s](l);
                        }
                    }
                }
            }

#pragma unroll
            for(int s = r; s < StencilSize; ++s)
            {
                int lo = min(local[r], local[s]);
                int hi = max(local[r], local[s]);
                H3x3s(I * HalfHessianSize + lo * (7 - lo) / 2 + hi)
                    .write(tet(local[r]), tet(local[s]), H[s]);
            }
        }
    }

    __global__ void StableNeoHookean3D_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Float>          mus,
        cuda_tool::CBufferView<Float>          lambdas,
        cuda_tool::CBufferView<Vector4i>       indices,
        cuda_tool::CBufferView<Vector3>        xs,
        cuda_tool::CBufferView<Matrix3x3>      Dm_invs,
        cuda_tool::DoubletVectorView<Float, 3> G3s,
        cuda_tool::TripletMatrixView<Float, 3> H3x3s,
        cuda_tool::CBufferView<Float>          volumes,
        Float                                  dt,
        bool                                   gradient_only,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        StableNeoHookean3D_gradient_hessian<false>(
            I, mus, lambdas, indices, xs, Dm_invs, G3s, {}, {}, H3x3s, volumes, dt, gradient_only);
    }

    __global__ void StableNeoHookean3D_compute_direct_gradient_hessian_kernel(
        cuda_tool::CBufferView<Float>          mus,
        cuda_tool::CBufferView<Float>          lambdas,
        cuda_tool::CBufferView<Vector4i>       indices,
        cuda_tool::CBufferView<Vector3>        xs,
        cuda_tool::CBufferView<Matrix3x3>      Dm_invs,
        cuda_tool::DenseVectorView<Float>      direct_gradients,
        cuda_tool::CBufferView<IndexT>         is_fixed,
        cuda_tool::TripletMatrixView<Float, 3> H3x3s,
        cuda_tool::CBufferView<Float>          volumes,
        Float                                  dt,
        int                                    n)
    {
        // a capped grid (ComputeGradientHessianInfo::block_cap()) loops
        for(int I = blockIdx.x * blockDim.x + threadIdx.x; I < n;
            I += gridDim.x * blockDim.x)
            StableNeoHookean3D_gradient_hessian<true>(
                I, mus, lambdas, indices, xs, Dm_invs, {}, direct_gradients, is_fixed, H3x3s, volumes, dt, false);
    }

    // The same, in blocks of Wide threads that still fill an SM each
    // (registers capped by the launch bounds): a capped grid that 256-thread
    // blocks would cover in two rounds, the second one latency-bound, can
    // cover the elements in one. The FP64 pipe bounds a full SM, so a round
    // takes time in proportion to the block width: the narrowest width that
    // covers the elements within the cap is launched.
    template <int Wide>
    __global__ void __launch_bounds__(Wide, 1)
        StableNeoHookean3D_compute_direct_gradient_hessian_wide_kernel(
            cuda_tool::CBufferView<Float>          mus,
            cuda_tool::CBufferView<Float>          lambdas,
            cuda_tool::CBufferView<Vector4i>       indices,
            cuda_tool::CBufferView<Vector3>        xs,
            cuda_tool::CBufferView<Matrix3x3>      Dm_invs,
            cuda_tool::DenseVectorView<Float>      direct_gradients,
            cuda_tool::CBufferView<IndexT>         is_fixed,
            cuda_tool::TripletMatrixView<Float, 3> H3x3s,
            cuda_tool::CBufferView<Float>          volumes,
            Float                                  dt,
            int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        StableNeoHookean3D_gradient_hessian<true>(
            I, mus, lambdas, indices, xs, Dm_invs, {}, direct_gradients, is_fixed, H3x3s, volumes, dt, false);
    }
}  // namespace

class StableNeoHookean3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 10;
    static constexpr SizeT StencilSize     = 4;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using FEM3DConstitution::FEM3DConstitution;

    vector<Float> h_mus;
    vector<Float> h_lambdas;

    cuda_tool::DeviceBuffer<Float> mus;
    cuda_tool::DeviceBuffer<Float> lambdas;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(mus.size());
        info.gradient_count(mus.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(mus.size() * HalfHessianSize);
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
                auto mu     = sc.tetrahedra().find<Float>("mu");
                auto lambda = sc.tetrahedra().find<Float>("lambda");

                return zip(mu->view(), lambda->view());
            },
            [&](const ForEachInfo& I, auto mu_and_lambda)
            {
                auto&& [mu, lambda] = mu_and_lambda;

                auto vI = I.global_index();

                h_mus[vI]     = mu;
                h_lambdas[vI] = lambda;
            });

        mus.resize(N);
        mus.view().copy_from(h_mus.data());

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        auto k = StableNeoHookean3D_do_compute_energy_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream()>>>(
                mus.cview(),
                lambdas.cview(),
                info.energies(),
                info.indices(),
                info.xs(),
                info.Dm_invs(),
                info.rest_volumes(),
                info.dt(),
                n);
        }
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        int n = (int)info.indices().size();
        if(n > 0 && info.direct_gradients_available() && !info.gradient_only())
        {
            // same launch shape as the doublet kernel, within the grid cap
            auto k = StableNeoHookean3D_compute_direct_gradient_hessian_kernel;
            auto d = StableNeoHookean3D_do_compute_gradient_hessian_kernel;
            int  grid = cuda_tool::best_grid_dim(n, d);
            if(info.block_cap() > 0 && grid > info.block_cap())
            {
                // the narrowest width (a warp at a time) that covers the
                // elements in one round; above 480 threads (15 warps) a
                // round is not shorter than two of the 256-thread blocks
                const int per_block = (n + info.block_cap() - 1) / info.block_cap();
                auto w = StableNeoHookean3D_compute_direct_gradient_hessian_wide_kernel<384>;
                int wide = 384;
                if(per_block > 448)
                {
                    w = StableNeoHookean3D_compute_direct_gradient_hessian_wide_kernel<480>;
                    wide = 480;
                }
                else if(per_block > 416)
                {
                    w = StableNeoHookean3D_compute_direct_gradient_hessian_wide_kernel<448>;
                    wide = 448;
                }
                else if(per_block > 384)
                {
                    w = StableNeoHookean3D_compute_direct_gradient_hessian_wide_kernel<416>;
                    wide = 416;
                }
                if(per_block <= wide)
                {
                    w<<<(n + wide - 1) / wide, wide, 0, info.stream()>>>(
                        mus.cview(),
                        lambdas.cview(),
                        info.indices(),
                        info.xs(),
                        info.Dm_invs(),
                        info.direct_gradients(),
                        info.is_fixed(),
                        info.hessians(),
                        info.rest_volumes(),
                        info.dt(),
                        n);
                    info.gradients_added_directly();
                    return;
                }
                grid = info.block_cap();
            }
            k<<<grid, cuda_tool::best_block_dim(d), 0, info.stream()>>>(
                mus.cview(),
                lambdas.cview(),
                info.indices(),
                info.xs(),
                info.Dm_invs(),
                info.direct_gradients(),
                info.is_fixed(),
                info.hessians(),
                info.rest_volumes(),
                info.dt(),
                n);
            info.gradients_added_directly();
            return;
        }
        auto k = StableNeoHookean3D_do_compute_gradient_hessian_kernel;
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream()>>>(
                mus.cview(),
                lambdas.cview(),
                info.indices(),
                info.xs(),
                info.Dm_invs(),
                info.gradients(),
                info.hessians(),
                info.rest_volumes(),
                info.dt(),
                info.gradient_only(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(StableNeoHookean3D);
}  // namespace uipc::backend::cuda
