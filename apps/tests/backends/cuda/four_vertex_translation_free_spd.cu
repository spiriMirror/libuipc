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

#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <utils/four_vertex_translation_free_spd.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
constexpr int PTCaseCount      = 7;
constexpr int EECaseCount      = 10;
constexpr int ParameterCount   = 3;
constexpr int ContactCaseCount = PTCaseCount + EECaseCount;
constexpr int CaseCount        = 1 + ParameterCount * ContactCaseCount;
constexpr int Dof              = 12;
// The production normal-contact Hessian path uses CTA8 / lane pitch 8.
constexpr int BlockSize = 8;
constexpr int LanePitch = 8;

template <typename T>
class DeviceAllocation
{
  public:
    explicit DeviceAllocation(std::size_t count)
    {
        CUDA_TOOL_CHECK(cudaMalloc(&m_data, count * sizeof(T)));
    }
    DeviceAllocation(const DeviceAllocation&)            = delete;
    DeviceAllocation& operator=(const DeviceAllocation&) = delete;
    ~DeviceAllocation()
    {
        if(m_data)
            cudaFree(m_data);
    }
    T* data() const { return m_data; }

  private:
    T* m_data = nullptr;
};

UIPC_DEVICE Float parameter_kappa(int parameter)
{
    return parameter == 0 ? 0.25 : parameter == 1 ? 2.0 : 64.0;
}

UIPC_DEVICE Float parameter_thickness(int parameter)
{
    return parameter == 0 ? 0.0 : parameter == 1 ? 0.05 : 0.075;
}

UIPC_DEVICE void make_synthetic(Matrix12x12& H)
{
    constexpr Float signs[4][3] = {
        {0.5, 0.5, 0.5}, {0.5, -0.5, -0.5}, {-0.5, 0.5, -0.5}, {-0.5, -0.5, 0.5}};
    Matrix9x9 relative;
#pragma unroll
    for(int row = 0; row < 9; ++row)
#pragma unroll
        for(int col = 0; col < 9; ++col)
            relative(row, col) =
                row == col ? Float(row - 4) : 0.03125 * Float(row + col + 1);

#pragma unroll
    for(int va = 0; va < 4; ++va)
#pragma unroll
        for(int aa = 0; aa < 3; ++aa)
#pragma unroll
            for(int vb = 0; vb < 4; ++vb)
#pragma unroll
                for(int ab = 0; ab < 3; ++ab)
                {
                    Float value = 0.0;
#pragma unroll
                    for(int ma = 0; ma < 3; ++ma)
#pragma unroll
                        for(int mb = 0; mb < 3; ++mb)
                            value += signs[va][ma] * relative(3 * ma + aa, 3 * mb + ab)
                                     * signs[vb][mb];
                    H(3 * va + aa, 3 * vb + ab) = value;
                }
}

UIPC_DEVICE void make_pt(Matrix12x12& H, int topology, int parameter)
{
    using namespace sym::codim_ipc_simplex_contact;
    const Vector4i flags[PTCaseCount] = {{1, 1, 0, 0},
                                         {1, 0, 1, 0},
                                         {1, 0, 0, 1},
                                         {1, 1, 1, 0},
                                         {1, 1, 0, 1},
                                         {1, 0, 1, 1},
                                         {1, 1, 1, 1}};
    const Vector3  P{0.0, 0.0, 0.1};
    const Vector3  T0{-1.0, -1.0, 0.0};
    const Vector3  T1{1.0, -1.0, 0.0};
    const Vector3  T2{0.0, 1.0, 0.0};
    Vector12       G;
    PT_barrier_gradient_hessian(
        G, H, flags[topology], parameter_kappa(parameter), 5.0, parameter_thickness(parameter), P, T0, T1, T2);
}

UIPC_DEVICE void make_ee(Matrix12x12& H, int topology, int parameter)
{
    using namespace sym::codim_ipc_simplex_contact;
    const Vector4i flags[EECaseCount - 1] = {{1, 0, 1, 0},
                                             {1, 0, 0, 1},
                                             {0, 1, 1, 0},
                                             {0, 1, 0, 1},
                                             {1, 0, 1, 1},
                                             {0, 1, 1, 1},
                                             {1, 1, 1, 0},
                                             {1, 1, 0, 1},
                                             {1, 1, 1, 1}};
    const Vector3  rest_a0{-1.0, 0.0, 0.1};
    const Vector3  rest_a1{1.0, 0.0, 0.1};
    const Vector3  rest_b0{0.0, -1.0, 0.0};
    const Vector3  rest_b1{0.0, 1.0, 0.0};
    Vector3        a0 = rest_a0;
    Vector3        a1 = rest_a1;
    Vector3        b0 = rest_b0;
    Vector3        b1 = rest_b1;
    Vector4i       flag;
    if(topology < EECaseCount - 1)
        flag = flags[topology];
    else
    {
        b0   = Vector3{-0.75, 0.2, 0.0};
        b1   = Vector3{0.75, 0.2, 0.0};
        flag = Vector4i{1, 0, 1, 1};
    }
    Vector12 G;
    mollified_EE_barrier_gradient_hessian(G,
                                          H,
                                          flag,
                                          parameter_kappa(parameter),
                                          5.0,
                                          parameter_thickness(parameter),
                                          rest_a0,
                                          rest_a1,
                                          rest_b0,
                                          rest_b1,
                                          a0,
                                          a1,
                                          b0,
                                          b1);
}

__global__ void generate_cases_kernel(Float* raw)
{
    const int index = int(blockIdx.x * blockDim.x + threadIdx.x);
    if(index >= CaseCount)
        return;
    Matrix12x12 H;
    if(index == 0)
        make_synthetic(H);
    else
    {
        const int contact   = index - 1;
        const int parameter = contact / ContactCaseCount;
        const int topology  = contact % ContactCaseCount;
        if(topology < PTCaseCount)
            make_pt(H, topology, parameter);
        else
            make_ee(H, topology - PTCaseCount, parameter);
    }
#pragma unroll
    for(int col = 0; col < Dof; ++col)
#pragma unroll
        for(int row = 0; row < Dof; ++row)
            raw[std::size_t(index) * Dof * Dof + col * Dof + row] = H(row, col);
}

template <bool Reduced, int Count = CaseCount>
__global__ void project_cases_kernel(const Float* raw, Float* projected, int* status)
{
    __shared__ Float workspace_storage[Dof * Dof * LanePitch];
    const int        index = int(blockIdx.x * blockDim.x + threadIdx.x);
    if(index >= Count)
        return;
    FixedBankSoAMap<Dof, LanePitch> H(workspace_storage + threadIdx.x);
    const std::size_t               offset = std::size_t(index) * Dof * Dof;
#pragma unroll
    for(int col = 0; col < Dof; ++col)
#pragma unroll
        for(int row = 0; row < Dof; ++row)
            H(row, col) = raw[offset + col * Dof + row];

    Vector12 eigenvalues;
    bool     success;
    if constexpr(Reduced)
        success =
            selfadjoint_evd_four_vertex_translation_free_fixed_bank<LanePitch>(H, eigenvalues);
    else
        success = selfadjoint_evd_fixed_bank_shared<Dof>(H, eigenvalues);

    bool finite = success;
#pragma unroll
    for(int col = 0; col < Dof; ++col)
#pragma unroll
        for(int row = 0; row < Dof; ++row)
        {
            Float value = 0.0;
#pragma unroll
            for(int k = 0; k < Dof; ++k)
                value = fma(H(row, k), eigenvalues(k) * H(col, k), value);
            projected[offset + col * Dof + row] = value;
            finite                              = finite && isfinite(value);
        }
    status[index] = finite ? 0 : 1;
}

Float max_translation_residual(const Float* matrix)
{
    Float result = 0.0;
    for(int row = 0; row < Dof; ++row)
        for(int axis = 0; axis < 3; ++axis)
        {
            Float value = 0.0;
            for(int vertex = 0; vertex < 4; ++vertex)
                value += matrix[(3 * vertex + axis) * Dof + row];
            result = std::max(result, std::abs(value));
        }
    return result;
}
}  // namespace

TEST_CASE("four-vertex normal-contact PSD reduction matches full EVD",
          "[cuda][contact][normal][translation_free_evd]")
{
    const std::size_t       matrix_values = std::size_t(CaseCount) * Dof * Dof;
    DeviceAllocation<Float> raw_device(matrix_values);
    DeviceAllocation<Float> full_device(matrix_values);
    DeviceAllocation<Float> reduced_device(matrix_values);
    DeviceAllocation<int>   full_status_device(CaseCount);
    DeviceAllocation<int>   reduced_status_device(CaseCount);

    generate_cases_kernel<<<(CaseCount + BlockSize - 1) / BlockSize, BlockSize>>>(
        raw_device.data());
    project_cases_kernel<false><<<(CaseCount + BlockSize - 1) / BlockSize, BlockSize>>>(
        raw_device.data(), full_device.data(), full_status_device.data());
    project_cases_kernel<true><<<(CaseCount + BlockSize - 1) / BlockSize, BlockSize>>>(
        raw_device.data(), reduced_device.data(), reduced_status_device.data());
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    std::vector<Float> raw(matrix_values);
    std::vector<Float> full(matrix_values);
    std::vector<Float> reduced(matrix_values);
    std::vector<int>   full_status(CaseCount);
    std::vector<int>   reduced_status(CaseCount);
    CUDA_TOOL_CHECK(cudaMemcpy(
        raw.data(), raw_device.data(), matrix_values * sizeof(Float), cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(
        full.data(), full_device.data(), matrix_values * sizeof(Float), cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(reduced.data(),
                               reduced_device.data(),
                               matrix_values * sizeof(Float),
                               cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(full_status.data(),
                               full_status_device.data(),
                               CaseCount * sizeof(int),
                               cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(reduced_status.data(),
                               reduced_status_device.data(),
                               CaseCount * sizeof(int),
                               cudaMemcpyDeviceToHost));

    for(int index = 0; index < CaseCount; ++index)
    {
        const Float* raw_matrix  = raw.data() + std::size_t(index) * Dof * Dof;
        const Float* full_matrix = full.data() + std::size_t(index) * Dof * Dof;
        const Float* reduced_matrix = reduced.data() + std::size_t(index) * Dof * Dof;
        Float max_abs = 0.0;
        Float scale   = 0.0;
        for(int i = 0; i < Dof * Dof; ++i)
        {
            max_abs = std::max(max_abs, std::abs(full_matrix[i] - reduced_matrix[i]));
            scale = std::max(
                scale, std::max(std::abs(full_matrix[i]), std::abs(reduced_matrix[i])));
        }
        const Float tolerance = 2e-9 * (1.0 + scale);
        INFO("case=" << index << " max_abs=" << max_abs << " scale=" << scale);
        REQUIRE(full_status[index] == 0);
        REQUIRE(reduced_status[index] == 0);
        REQUIRE(max_translation_residual(raw_matrix) <= 1e-10 * (1.0 + scale));
        REQUIRE(max_translation_residual(reduced_matrix) <= tolerance);
        REQUIRE(max_abs <= tolerance);
    }
}

TEST_CASE("four-vertex PSD reduction avoids overflow in finite Hadamard sums",
          "[cuda][contact][normal][translation_free_evd]")
{
    // The input entries are at most 3e307, and the nonzero eigenvalue is
    // +/-1.2e308. Both are representable. Summing before multiplying by 0.5
    // in the second Hadamard transform nevertheless used to form +/-2.4e308.
    constexpr int Count              = 4;
    const Float   eigenvalues[Count] = {1.2e308, -1.2e308, 1e300, -1e300};
    Vector12      mode               = Vector12::Zero();
    mode(0) = mode(3) = 0.5;
    mode(6) = mode(9)                = -0.5;
    const Matrix12x12  projector     = mode * mode.transpose();
    const std::size_t  matrix_values = std::size_t(Count) * Dof * Dof;
    std::vector<Float> raw(matrix_values);
    for(int index = 0; index < Count; ++index)
        for(int col = 0; col < Dof; ++col)
            for(int row = 0; row < Dof; ++row)
                raw[std::size_t(index) * Dof * Dof + col * Dof + row] =
                    eigenvalues[index] * projector(row, col);

    DeviceAllocation<Float> raw_device(matrix_values);
    DeviceAllocation<Float> full_device(matrix_values);
    DeviceAllocation<Float> reduced_device(matrix_values);
    DeviceAllocation<int>   full_status_device(Count);
    DeviceAllocation<int>   reduced_status_device(Count);
    CUDA_TOOL_CHECK(cudaMemcpy(
        raw_device.data(), raw.data(), matrix_values * sizeof(Float), cudaMemcpyHostToDevice));
    project_cases_kernel<false, Count><<<1, BlockSize>>>(
        raw_device.data(), full_device.data(), full_status_device.data());
    project_cases_kernel<true, Count><<<1, BlockSize>>>(
        raw_device.data(), reduced_device.data(), reduced_status_device.data());
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    std::vector<Float> full(matrix_values);
    std::vector<Float> reduced(matrix_values);
    std::vector<int>   full_status(Count);
    std::vector<int>   reduced_status(Count);
    CUDA_TOOL_CHECK(cudaMemcpy(
        full.data(), full_device.data(), matrix_values * sizeof(Float), cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(reduced.data(),
                               reduced_device.data(),
                               matrix_values * sizeof(Float),
                               cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(
        full_status.data(), full_status_device.data(), Count * sizeof(int), cudaMemcpyDeviceToHost));
    CUDA_TOOL_CHECK(cudaMemcpy(reduced_status.data(),
                               reduced_status_device.data(),
                               Count * sizeof(int),
                               cudaMemcpyDeviceToHost));

    for(int index = 0; index < Count; ++index)
    {
        INFO("eigenvalue=" << eigenvalues[index]);
        REQUIRE(full_status[index] == 0);
        REQUIRE(reduced_status[index] == 0);
        const Float magnitude = std::abs(eigenvalues[index]);
        const Matrix12x12 expected = (eigenvalues[index] > 0.0 ? 1.0 : 0.0) * projector;
        // Normalize before differences, sums, or norms, so the oracle itself
        // cannot overflow merely because the finite Hessian is large.
        const Matrix12x12 full_normalized =
            Eigen::Map<const Matrix12x12>(full.data() + index * Dof * Dof) / magnitude;
        const Matrix12x12 reduced_normalized =
            Eigen::Map<const Matrix12x12>(reduced.data() + index * Dof * Dof) / magnitude;
        REQUIRE(full_normalized.allFinite());
        REQUIRE(reduced_normalized.allFinite());
        REQUIRE((full_normalized - expected).cwiseAbs().maxCoeff() <= 2e-12);
        REQUIRE((reduced_normalized - expected).cwiseAbs().maxCoeff() <= 2e-12);
        REQUIRE((reduced_normalized - full_normalized).cwiseAbs().maxCoeff() <= 2e-12);
        REQUIRE((reduced_normalized - reduced_normalized.transpose()).cwiseAbs().maxCoeff()
                <= 2e-12);
        REQUIRE(max_translation_residual(reduced_normalized.data()) <= 2e-12);
        Eigen::SelfAdjointEigenSolver<Matrix12x12> solver(reduced_normalized);
        REQUIRE(solver.info() == Eigen::Success);
        REQUIRE(solver.eigenvalues().minCoeff() >= -2e-12);
    }
}
