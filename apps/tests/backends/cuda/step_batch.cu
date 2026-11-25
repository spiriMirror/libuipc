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

// StepBatch (the step of a line-search trial in one kernel) against kernels
// that compute each range the way the separate step_forward kernels do: the
// results must be the same bytes, and fixed bodies must keep their DOFs.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <line_search/step_batch.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using uipc::Float;
using uipc::IndexT;
using uipc::Vector12;
using uipc::Vector3;
using uipc::backend::cuda::StepBatch;

namespace
{
__global__ void step_batch_test_positions_kernel(
    Vector3* x, const Vector3* x0, const Vector3* dx, Float alpha, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < n)
        x[i] = uipc::backend::cuda::step_position(x0[i], alpha, dx[i]);
}

__global__ void step_batch_test_dofs12_kernel(const IndexT*   fixed,
                                              const Vector12* q0,
                                              Vector12*       q,
                                              const Vector12* dq,
                                              Float           alpha,
                                              int             n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n || fixed[i])
        return;
    q[i] = q0[i] + alpha * dq[i];
}

__global__ void step_batch_test_dofs3_kernel(
    const Vector3* x0, Vector3* x, const Vector3* dx, Float alpha, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < n)
        x[i] = x0[i] + alpha * dx[i];
}

template <typename T>
std::vector<T> random_values(size_t n, std::mt19937_64& rng)
{
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::uniform_int_distribution<int>     e(-20, 20);
    std::vector<T>                         v(n);
    for(auto& x : v)
        for(int k = 0; k < x.size(); ++k)
            x[k] = std::ldexp(u(rng), e(rng));
    return v;
}

template <typename T>
std::vector<T> host_of(const cuda_tool::DeviceBuffer<T>& d)
{
    std::vector<T> h;
    d.copy_to(h);
    return h;
}

template <typename T>
bool same_bytes(const std::vector<T>& a, const std::vector<T>& b)
{
    return a.size() == b.size()
           && std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
}
}  // namespace

TEST_CASE("StepBatch steps like the separate step_forward kernels", "[cuda][line_search][step_batch]")
{
    std::mt19937_64 rng(29);
    const int       nv = 3001, nb = 77, nx = 1234;
    auto h_x0 = random_values<Vector3>(nv, rng), h_dx = random_values<Vector3>(nv, rng);
    auto h_q0 = random_values<Vector12>(nb, rng), h_dq = random_values<Vector12>(nb, rng);
    auto h_y0 = random_values<Vector3>(nx, rng), h_dy = random_values<Vector3>(nx, rng);
    std::vector<IndexT> h_fixed(nb, 0);
    for(int i = 0; i < nb; i += 5)
        h_fixed[i] = 1;
    auto h_q_old = random_values<Vector12>(nb, rng);  // what fixed bodies must keep

    cuda_tool::DeviceBuffer<Vector3> x0, dx, x_batch, x_ref, y0, dy, y_batch, y_ref;
    cuda_tool::DeviceBuffer<Vector12> q0, dq, q_batch, q_ref;
    cuda_tool::DeviceBuffer<IndexT>   fixed;
    x0.copy_from(h_x0.data(), nv);
    dx.copy_from(h_dx.data(), nv);
    y0.copy_from(h_y0.data(), nx);
    dy.copy_from(h_dy.data(), nx);
    q0.copy_from(h_q0.data(), nb);
    dq.copy_from(h_dq.data(), nb);
    fixed.copy_from(h_fixed.data(), nb);
    x_batch.resize(nv);
    x_ref.resize(nv);
    y_batch.resize(nx);
    y_ref.resize(nx);
    q_batch.copy_from(h_q_old.data(), nb);
    q_ref.copy_from(h_q_old.data(), nb);

    for(Float alpha : {1.0, 0.5, 0.123456789, 1.0 / 3.0})
    {
        StepBatch steps{alpha};
        steps.positions(x_batch.view(), x0.cview(), dx.cview());
        steps.dofs(q_batch.view(), q0.cview(), dq.cview(), fixed.cview(), nb);
        steps.dofs(y_batch.view(), y0.cview(), dy.cview());
        steps.launch();

        step_batch_test_positions_kernel<<<(nv + 127) / 128, 128>>>(
            x_ref.data(), x0.data(), dx.data(), alpha, nv);
        step_batch_test_dofs12_kernel<<<(nb + 127) / 128, 128>>>(
            fixed.data(), q0.data(), q_ref.data(), dq.data(), alpha, nb);
        step_batch_test_dofs3_kernel<<<(nx + 127) / 128, 128>>>(
            y0.data(), y_ref.data(), dy.data(), alpha, nx);
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());

        REQUIRE(same_bytes(host_of(x_batch), host_of(x_ref)));
        REQUIRE(same_bytes(host_of(q_batch), host_of(q_ref)));
        REQUIRE(same_bytes(host_of(y_batch), host_of(y_ref)));
    }
    auto q = host_of(q_batch);
    for(int i = 0; i < nb; i += 5)
        REQUIRE(std::memcmp(&q[i], &h_q_old[i], sizeof(Vector12)) == 0);
}

TEST_CASE("StepBatch rejects mismatched and too many ranges", "[cuda][line_search][step_batch]")
{
    cuda_tool::DeviceBuffer<Vector3> a, b;
    a.resize(8);
    b.resize(8);
    StepBatch steps{0.5};
    REQUIRE_THROWS(steps.dofs(a.view(0, 3), b.cview(0, 4), b.cview(0, 3)));
    for(int i = 0; i < 8; ++i)
        steps.dofs(a.view(i, 1), b.cview(i, 1), b.cview(i, 1));
    REQUIRE_THROWS(steps.dofs(a.view(0, 1), b.cview(0, 1), b.cview(0, 1)));
    steps.launch();
    // an empty batch launches nothing
    steps.launch();
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
}
