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
#include <linear_system/fused_pcg_kernels.h>
#include <cuda_tool/graph.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

#if CUDA_TOOL_GRAPH_WHILE
namespace
{
// Stand-in for the update/apply/dot step of iteration k = iter + 1: adds
// rz_k = rz0 * 2^-k to the slot's accumulator and, like the real step, does
// nothing once the solve state is terminal.
__global__ void while_publication_test_dot_kernel(cuda_tool::CDense<IndexT> converged,
                                                  cuda_tool::CDense<IndexT> iter,
                                                  cuda_tool::Dense<Float> rz_accum,
                                                  Float rz0)
{
    if(*converged != 0)
        return;
    const int k = (int)(*iter) + 1;
    *rz_accum += ::ldexp(rz0, -k);
}

__global__ void while_publication_test_reset_kernel(cuda_tool::Dense<IndexT> converged,
                                                    cuda_tool::Dense<IndexT> iter,
                                                    cuda_tool::Dense<IndexT> iter0,
                                                    cuda_tool::Dense<Float> rz,
                                                    cuda_tool::Dense<Float> rz_accum0,
                                                    Float rz0)
{
    *converged = 0;
    *iter      = 0;
    *iter0     = 0;
    *rz        = rz0;
    *rz_accum0 = 0;
}

struct Outcome
{
    IndexT             iter      = 0;
    IndexT             state     = 0;
    Float              published = 0;
    Float              rz        = 0;
    std::vector<Float> p;
};

constexpr int   ScalarCount = 3 * 1024 + 7;  // several CTAs and a tail
constexpr Float Rz0         = 1.0;  // rz_k = 2^-k, exact on host and device

Float p0(int i)
{
    return 0.01 * (i % 13 + 1);
}
Float z0(int i)
{
    return 0.02 * (i % 11 + 1);
}

// Host model of the plain loop: converged once |rz_k| <= tol, at most
// max_iter_minus_1 iterations, p = z + (rz_k / rz_{k-1}) p while running.
Outcome expected_outcome(int max_iter_minus_1, Float tol)
{
    Outcome out;
    out.p.resize(ScalarCount);
    for(int i = 0; i < ScalarCount; ++i)
        out.p[i] = p0(i);
    Float rz = Rz0;
    for(int k = 1;; ++k)
    {
        const Float value = std::ldexp(Rz0, -k);
        out.iter          = k;
        out.published     = value;
        if(std::abs(value) <= tol)
        {
            out.state = 1;
            break;
        }
        if(!(k < max_iter_minus_1))
        {
            out.state = 2;
            break;
        }
        const Float beta = value / rz;
        rz               = value;
        for(int i = 0; i < ScalarCount; ++i)
            out.p[i] = z0(i) + beta * out.p[i];
    }
    out.rz = rz;
    return out;
}

Outcome run_while_graph(int max_iter_minus_1, Float tol, int launches)
{
    std::vector<Float> p(ScalarCount), z(ScalarCount);
    for(int i = 0; i < ScalarCount; ++i)
    {
        p[i] = p0(i);
        z[i] = z0(i);
    }
    cuda_tool::DeviceDenseVector<Float> d_p, d_z;
    d_p.resize(ScalarCount);
    d_z.resize(ScalarCount);
    d_z.buffer_view().copy_from(z.data());

    std::array<cuda_tool::DeviceVar<Float>, 2>  d_rz;  // per body slot
    std::array<cuda_tool::DeviceVar<Float>, 2>  d_rz_accum;
    cuda_tool::DeviceVar<Float>                 d_published{Float{-1}};
    cuda_tool::DeviceVar<Float>                 d_beta{Float{0}};
    cuda_tool::DeviceVar<IndexT>                d_converged{IndexT{0}};
    cuda_tool::DeviceVar<Float>                 d_rz_tol{tol};
    cuda_tool::DeviceVar<IndexT>                d_iter{IndexT{0}};
    cuda_tool::DeviceVar<IndexT>                d_state{IndexT{-1}};
    std::array<cuda_tool::DeviceVar<IndexT>, 2> d_iter_slot;  // per body slot

    cuda_tool::GraphWhile graph;
    auto                  result = graph.capture(
        [&](cudaStream_t stream, cudaGraphConditionalHandle)
        {
            while_publication_test_reset_kernel<<<1, 1, 0, stream>>>(
                d_converged.viewer(),
                d_iter.viewer(),
                d_iter_slot[0].viewer(),
                d_rz[0].viewer(),
                d_rz_accum[0].viewer(),
                Rz0);
        },
        [&](cudaStream_t stream, cudaGraphConditionalHandle handle)
        {
            // Two iterations per body, one per ping-pong slot, as in
            // LinearFusedPCG's while-loop graph.
            for(SizeT slot = 0; slot < 2; ++slot)
            {
                while_publication_test_dot_kernel<<<1, 1, 0, stream>>>(
                    d_converged.cviewer(), d_iter.cviewer(), d_rz_accum[slot].viewer(), Rz0);
                launch_fused_pcg_while_update_direction(d_p.view(),
                                                        d_z.cview(),
                                                        d_rz[slot].view(),
                                                        d_rz[slot ^ 1].view(),
                                                        d_rz_accum[slot].view(),
                                                        d_published.view(),
                                                        d_rz_accum[slot ^ 1].view(),
                                                        d_beta.view(),
                                                        d_converged.view(),
                                                        d_rz_tol.view(),
                                                        d_iter_slot[slot].view(),
                                                        d_iter_slot[slot ^ 1].view(),
                                                        d_iter.view(),
                                                        d_state.view(),
                                                        max_iter_minus_1,
                                                        handle,
                                                        stream);
            }
        });
    REQUIRE(result == cuda_tool::GraphWhile::Result::Ok);

    Outcome out;
    for(int launch = 0; launch < launches; ++launch)
    {
        // Each launch must restart from the setup chain's state.
        d_p.buffer_view().copy_from(p.data());
        REQUIRE(graph.launch_sync() == cudaSuccess);
    }
    out.iter      = d_iter;
    out.state     = d_converged;
    out.published = d_published;
    // the publisher's state_out ends equal to the shared state
    REQUIRE(IndexT(d_state) == out.state);
    // iteration k runs on slot (k - 1) % 2 and reads rz_{k-1} there; the
    // terminal one publishes no further rz
    out.rz = d_rz[(out.iter - 1) % 2];
    out.p.resize(ScalarCount);
    d_p.buffer_view().copy_to(out.p.data());
    return out;
}

// The former WHILE direction step (claim/countdown publication with one rz),
// kept as the bitwise reference of the slot-buffered one.
constexpr IndexT RefRunning = 0, RefConverged = 1, RefStopped = 2;
constexpr IndexT RefClaimed = std::numeric_limits<IndexT>::min();

__global__ void reference_while_direction_kernel(cuda_tool::DenseVectorView<Float> p,
                                                 cuda_tool::CDenseVectorView<Float> z,
                                                 cuda_tool::Dense<Float> rz,
                                                 cuda_tool::CDense<Float> rz_accum,
                                                 cuda_tool::Dense<Float> published_rz_new,
                                                 cuda_tool::Dense<Float> next_rz_accum,
                                                 cuda_tool::Dense<Float> beta,
                                                 cuda_tool::Dense<IndexT> converged,
                                                 cuda_tool::CDense<Float> rz_tol,
                                                 cuda_tool::Dense<IndexT> iter,
                                                 int max_iter_minus_1,
                                                 cudaGraphConditionalHandle handle,
                                                 int n)
{
    __shared__ Float  block_beta;
    __shared__ IndexT block_updates_p;

    if(threadIdx.x == 0)
    {
        block_updates_p = 0;
        IndexT observed = atomicCAS(converged.data(), RefRunning, RefClaimed);
        if(observed == RefRunning)
        {
            const Float old_value = *rz;
            const Float new_value = *rz_accum;
            *published_rz_new     = new_value;
            const bool is_conv    = ::abs(new_value) <= *rz_tol;
            const int  k          = (int)(*iter) + 1;
            *iter                 = k;
            const bool keep       = !is_conv && (k < max_iter_minus_1);
            cudaGraphSetConditional(handle, keep ? 1u : 0u);
            const IndexT state = is_conv ? RefConverged : (keep ? RefRunning : RefStopped);
            if(state != RefRunning)
            {
                __threadfence();
                atomicExch(converged.data(), state);
                observed = state;
            }
            else
            {
                *beta          = new_value / old_value;
                *rz            = new_value;
                *next_rz_accum = Float{0};
                __threadfence();
                const IndexT countdown = -static_cast<IndexT>(gridDim.x);
                atomicExch(converged.data(), countdown);
                observed = countdown;
            }
        }
        else
        {
            while(observed == RefClaimed)
                observed = atomicAdd(converged.data(), IndexT{0});
        }
        if(observed < 0 && observed != RefClaimed)
        {
            __threadfence();
            block_beta      = *beta;
            block_updates_p = 1;
        }
    }
    __syncthreads();

    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(block_updates_p && i < n)
        p(i) = z(i) + block_beta * p(i);

    __syncthreads();
    if(threadIdx.x == 0 && block_updates_p)
        atomicAdd(converged.data(), IndexT{1});
}

// The slot-buffered WHILE direction step as it was before z and p were read
// ahead of the barrier and the state was published next to the count:
// the bitwise reference of the current one.
struct FormerWhileDecision
{
    cuda_tool::CDense<Float>   rz_tol;
    cuda_tool::CDense<IndexT>  iter_cur;
    cuda_tool::Dense<IndexT>   iter_next;
    cuda_tool::Dense<IndexT>   iter_out;
    int                        max_iter_minus_1;
    cudaGraphConditionalHandle handle;

    __device__ bool running(Float rz_new) const
    {
        const int k = (int)(*iter_cur) + 1;
        return !(::abs(rz_new) <= *rz_tol) && k < max_iter_minus_1;
    }

    __device__ IndexT operator()(Float rz_new) const
    {
        const bool converged = ::abs(rz_new) <= *rz_tol;
        const int  k         = (int)(*iter_cur) + 1;
        *iter_next           = k;
        *iter_out            = k;
        const bool keep      = !converged && (k < max_iter_minus_1);
        cudaGraphSetConditional(handle, keep ? 1u : 0u);
        if(converged)
            return RefConverged;
        return keep ? RefRunning : RefStopped;
    }
};

__global__ void former_while_direction_kernel(cuda_tool::DenseVectorView<Float> p,
                                              cuda_tool::CDenseVectorView<Float> z,
                                              cuda_tool::CDense<Float> rz_cur,
                                              cuda_tool::Dense<Float>  rz_next,
                                              cuda_tool::CDense<Float> rz_accum,
                                              cuda_tool::Dense<Float> published_rz_new,
                                              cuda_tool::Dense<Float> next_rz_accum,
                                              cuda_tool::Dense<Float> beta,
                                              cuda_tool::Dense<IndexT> converged,
                                              FormerWhileDecision decide,
                                              int                 n)
{
    __shared__ Float block_beta;
    __shared__ int   block_updates_p;

    if(threadIdx.x == 0)
    {
        block_updates_p = 0;
        const IndexT state = *static_cast<volatile const IndexT*>(converged.data());
        if(state == RefRunning)
        {
            const Float old_value = *rz_cur;
            const Float new_value = *rz_accum;
            bool        running   = false;
            if(blockIdx.x == 0)
            {
                *published_rz_new = new_value;
                const IndexT next = decide(new_value);
                running           = next == RefRunning;
                if(running)
                {
                    *beta          = new_value / old_value;
                    *rz_next       = new_value;
                    *next_rz_accum = Float{0};
                }
                else
                {
                    atomicExch(converged.data(), next);
                }
            }
            else
            {
                running = decide.running(new_value);
            }
            if(running)
            {
                block_beta      = new_value / old_value;
                block_updates_p = 1;
            }
        }
    }
    __syncthreads();

    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(block_updates_p && i < n)
        p(i) = z(i) + block_beta * p(i);
}

// rz_k = rz0 * 0.37^k: betas with rounding in every iteration
__global__ void rounding_test_dot_kernel(cuda_tool::CDense<IndexT> converged,
                                         cuda_tool::CDense<IndexT> iter,
                                         cuda_tool::Dense<Float>   rz_accum)
{
    if(*converged != 0)
        return;
    const int k = (int)(*iter) + 1;
    *rz_accum += ::pow(0.37, (double)k) * 1.3;
}

struct BitOutcome
{
    IndexT             iter = 0, state = 0;
    Float              published = 0;
    std::vector<Float> p;
};

// mode 0: the claim/countdown kernel, 1: the former slot-buffered kernel,
// 2: the library's launch
BitOutcome run_rounding_graph(int mode, int max_iter_minus_1, Float tol)
{
    std::vector<Float> p(ScalarCount), z(ScalarCount);
    for(int i = 0; i < ScalarCount; ++i)
    {
        p[i] = std::sin(0.1 * i) * 1e-3 + p0(i);
        z[i] = std::cos(0.07 * i) * 3e-2 + z0(i);
    }
    cuda_tool::DeviceDenseVector<Float> d_p, d_z;
    d_p.resize(ScalarCount);
    d_z.resize(ScalarCount);
    d_z.buffer_view().copy_from(z.data());

    std::array<cuda_tool::DeviceVar<Float>, 2>  d_rz, d_rz_accum;
    std::array<cuda_tool::DeviceVar<IndexT>, 2> d_iter_slot;
    cuda_tool::DeviceVar<Float>                 d_published{Float{-1}};
    cuda_tool::DeviceVar<Float>                 d_beta{Float{0}};
    cuda_tool::DeviceVar<IndexT>                d_converged{IndexT{0}};
    cuda_tool::DeviceVar<Float>                 d_rz_tol{tol};
    cuda_tool::DeviceVar<IndexT>                d_iter{IndexT{0}};
    cuda_tool::DeviceVar<IndexT>                d_state{IndexT{-1}};

    const int             n    = ScalarCount;
    const int             grid = (n + 255) / 256;
    cuda_tool::GraphWhile graph;
    auto                  result = graph.capture(
        [&](cudaStream_t stream, cudaGraphConditionalHandle)
        {
            while_publication_test_reset_kernel<<<1, 1, 0, stream>>>(
                d_converged.viewer(),
                d_iter.viewer(),
                d_iter_slot[0].viewer(),
                d_rz[0].viewer(),
                d_rz_accum[0].viewer(),
                1.3);
        },
        [&](cudaStream_t stream, cudaGraphConditionalHandle handle)
        {
            for(SizeT slot = 0; slot < 2; ++slot)
            {
                rounding_test_dot_kernel<<<1, 1, 0, stream>>>(
                    d_converged.cviewer(), d_iter.cviewer(), d_rz_accum[slot].viewer());
                if(mode == 0)
                    reference_while_direction_kernel<<<grid, 256, 0, stream>>>(
                        d_p.view(),
                        d_z.cview(),
                        d_rz[0].viewer(),
                        d_rz_accum[slot].cviewer(),
                        d_published.viewer(),
                        d_rz_accum[slot ^ 1].viewer(),
                        d_beta.viewer(),
                        d_converged.viewer(),
                        d_rz_tol.cviewer(),
                        d_iter.viewer(),
                        max_iter_minus_1,
                        handle,
                        n);
                else if(mode == 1)
                    former_while_direction_kernel<<<grid, 256, 0, stream>>>(
                        d_p.view(),
                        d_z.cview(),
                        d_rz[slot].cviewer(),
                        d_rz[slot ^ 1].viewer(),
                        d_rz_accum[slot].cviewer(),
                        d_published.viewer(),
                        d_rz_accum[slot ^ 1].viewer(),
                        d_beta.viewer(),
                        d_converged.viewer(),
                        FormerWhileDecision{d_rz_tol.cviewer(),
                                            d_iter_slot[slot].cviewer(),
                                            d_iter_slot[slot ^ 1].viewer(),
                                            d_iter.viewer(),
                                            max_iter_minus_1,
                                            handle},
                        n);
                else
                    launch_fused_pcg_while_update_direction(
                        d_p.view(),
                        d_z.cview(),
                        d_rz[slot].view(),
                        d_rz[slot ^ 1].view(),
                        d_rz_accum[slot].view(),
                        d_published.view(),
                        d_rz_accum[slot ^ 1].view(),
                        d_beta.view(),
                        d_converged.view(),
                        d_rz_tol.view(),
                        d_iter_slot[slot].view(),
                        d_iter_slot[slot ^ 1].view(),
                        d_iter.view(),
                        d_state.view(),
                        max_iter_minus_1,
                        handle,
                        stream);
            }
        });
    REQUIRE(result == cuda_tool::GraphWhile::Result::Ok);
    d_p.buffer_view().copy_from(p.data());
    REQUIRE(graph.launch_sync() == cudaSuccess);

    BitOutcome out;
    out.iter      = d_iter;
    out.state     = d_converged;
    out.published = d_published;
    if(mode == 2)  // the publisher's state_out ends equal to the shared state
        REQUIRE(IndexT(d_state) == out.state);
    out.p.resize(ScalarCount);
    d_p.buffer_view().copy_to(out.p.data());
    return out;
}

void require_outcome(const Outcome& actual, const Outcome& expected)
{
    REQUIRE(actual.iter == expected.iter);
    REQUIRE(actual.state == expected.state);
    REQUIRE(actual.published == expected.published);
    REQUIRE(actual.rz == expected.rz);
    REQUIRE(actual.p.size() == expected.p.size());
    for(SizeT i = 0; i < actual.p.size(); ++i)
        REQUIRE(actual.p[i] == Catch::Approx(expected.p[i]).margin(1e-14));
}
}  // namespace
#endif

TEST_CASE("Fused PCG while-loop publication reproduces the loop decisions",
          "[build_solve_focused][fused_pcg][while_publication]")
{
#if CUDA_TOOL_GRAPH_WHILE
    // conditional graph nodes need a CUDA >= 12.4 driver
    if(!cuda_tool::GraphWhile::runtime_supported())
        return;

    // rz_k = 2^-k: tol = 2^-K converges exactly at iteration K. Odd and even
    // K end in the first or the second slot of a two-iteration body.
    for(int converge_at : {1, 2, 3, 4})
    {
        CAPTURE(converge_at);
        const Float tol = std::ldexp(Rz0, -converge_at);
        require_outcome(run_while_graph(100, tol, 2), expected_outcome(100, tol));
        REQUIRE(expected_outcome(100, tol).iter == converge_at);
    }

    // Without convergence the loop stops after max_iter-1 iterations with
    // the Stopped state, in either slot.
    for(int max_iter_minus_1 : {1, 5, 6})
    {
        CAPTURE(max_iter_minus_1);
        const auto expected = expected_outcome(max_iter_minus_1, Float{0});
        REQUIRE(expected.iter == max_iter_minus_1);
        REQUIRE(expected.state == 2);
        require_outcome(run_while_graph(max_iter_minus_1, Float{0}, 2), expected);
    }

    // Convergence wins over the iteration limit on the same iteration.
    {
        const Float tol      = std::ldexp(Rz0, -5);
        const auto  expected = expected_outcome(5, tol);
        REQUIRE(expected.state == 1);
        require_outcome(run_while_graph(5, tol, 1), expected);
    }
#endif
}

TEST_CASE("Fused PCG while-loop direction step keeps the bits of the claim/countdown publication",
          "[build_solve_focused][fused_pcg][while_publication]")
{
#if CUDA_TOOL_GRAPH_WHILE
    if(!cuda_tool::GraphWhile::runtime_supported())
        return;

    // converged in either slot, stopped by the iteration limit in either
    for(auto [max_iter_minus_1, tol] : std::array<std::pair<int, Float>, 4>{
            {{100, 1e-4}, {100, 3e-4}, {7, Float{0}}, {8, Float{0}}}})
    {
        CAPTURE(max_iter_minus_1, tol);
        const BitOutcome ref = run_rounding_graph(0, max_iter_minus_1, tol);
        const BitOutcome now = run_rounding_graph(2, max_iter_minus_1, tol);
        REQUIRE(ref.iter > 2);
        REQUIRE(now.iter == ref.iter);
        REQUIRE(now.state == ref.state);
        REQUIRE(std::memcmp(&now.published, &ref.published, sizeof(Float)) == 0);
        REQUIRE(std::memcmp(now.p.data(), ref.p.data(), now.p.size() * sizeof(Float)) == 0);
        // and those of the slot-buffered kernel before its early z/p loads
        const BitOutcome former = run_rounding_graph(1, max_iter_minus_1, tol);
        REQUIRE(now.iter == former.iter);
        REQUIRE(now.state == former.state);
        REQUIRE(std::memcmp(&now.published, &former.published, sizeof(Float)) == 0);
        REQUIRE(std::memcmp(now.p.data(), former.p.data(), now.p.size() * sizeof(Float)) == 0);
    }
#endif
}
