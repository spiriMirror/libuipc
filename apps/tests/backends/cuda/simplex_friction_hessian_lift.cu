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

#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <utils/make_spd.h>

#include "eigen_reference.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
namespace contact = uipc::backend::cuda::sym::codim_ipc_contact;

constexpr Float EpsVh       = 0.125;
constexpr int   RadiusCount = 4;
constexpr int   ScaleCount  = 3;
constexpr int   CaseCount   = RadiusCount * ScaleCount;
constexpr int   MaxDof      = 12;

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

struct Result
{
    Float optimized[MaxDof * MaxDof];
    Float legacy[MaxDof * MaxDof];
    Float signed_scale;
    int   dof;
    int   direct;
};

UIPC_DEVICE Float radius_for(int index)
{
    switch(index % RadiusCount)
    {
        case 0:
            return 0.0;
        case 1:
            return 0.5 * EpsVh;
        case 2:
            return EpsVh;
        default:
            return 2.0 * EpsVh;
    }
}

UIPC_DEVICE Float scale_for(int index)
{
    switch(index / RadiusCount)
    {
        case 0:
            return 0.5;
        case 1:
            return 0.0;
        default:
            return -0.5;
    }
}

template <int N>
__global__ void friction_hessian_lift_kernel(Result* results)
{
    const int index = static_cast<int>(threadIdx.x);
    if(index >= CaseCount)
        return;

    const Float   radius       = radius_for(index);
    const Float   signed_scale = scale_for(index);
    const Vector2 tangent{radius, 0.0};

    Matrix2x2 H2;
    contact::friction_hessian(H2, signed_scale, 1.0, EpsVh, tangent);

    Matrix<Float, 2, N> J;
    for(int col = 0; col < N; ++col)
    {
        J(0, col) = 0.03125 * static_cast<Float>(col + 1);
        J(1, col) = ((col & 1) ? -0.046875 : 0.0625) * static_cast<Float>((col % 3) + 1);
    }

    Matrix<Float, N, N> optimized;
    const bool direct = contact::lift_friction_hessian(optimized, J, H2, signed_scale);

    Matrix<Float, N, N> legacy = J.transpose() * H2 * J;
    uipc::backend::cuda::make_spd<N>(legacy);
    if(!direct)
        uipc::backend::cuda::make_spd<N>(optimized);

    auto& result        = results[index];
    result.signed_scale = signed_scale;
    result.dof          = N;
    result.direct       = direct ? 1 : 0;
    for(int row = 0; row < N; ++row)
    {
        for(int col = 0; col < N; ++col)
        {
            result.optimized[row * MaxDof + col] = optimized(row, col);
            result.legacy[row * MaxDof + col]    = legacy(row, col);
        }
    }
}

bool near(Float actual, Float expected)
{
    constexpr Float AbsTol = 2.0e-9;
    constexpr Float RelTol = 2.0e-9;
    const Float     scale  = std::max(std::abs(actual), std::abs(expected));
    return std::abs(actual - expected) <= AbsTol + RelTol * scale;
}

template <int N>
void validate_lift()
{
    DeviceAllocation<Result> device_results(CaseCount);
    friction_hessian_lift_kernel<N><<<1, CaseCount>>>(device_results.data());
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    std::vector<Result> results(CaseCount);
    CUDA_TOOL_CHECK(cudaMemcpy(results.data(),
                               device_results.data(),
                               results.size() * sizeof(Result),
                               cudaMemcpyDeviceToHost));

    for(int index = 0; index < CaseCount; ++index)
    {
        const auto& result = results[index];
        INFO("dof=" << N << ", case=" << index << ", signed_scale=" << result.signed_scale);
        REQUIRE(result.dof == N);
        REQUIRE(std::isfinite(result.signed_scale));
        REQUIRE(result.direct == (result.signed_scale >= 0.0 ? 1 : 0));

        Matrix<Float, N, N> optimized;
        Float               max_abs = 0.0;
        for(int row = 0; row < N; ++row)
        {
            for(int col = 0; col < N; ++col)
            {
                const Float actual = result.optimized[row * MaxDof + col];
                const Float legacy = result.legacy[row * MaxDof + col];
                REQUIRE(std::isfinite(actual));
                REQUIRE(std::isfinite(legacy));
                REQUIRE(near(actual, legacy));
                optimized(row, col) = actual;
                max_abs             = std::max(max_abs, std::abs(actual));
            }
        }

        const Matrix<Float, N, N> symmetric = 0.5 * (optimized + optimized.transpose());
        Matrix<Float, N, 1> eigenvalues;
        REQUIRE(uipc::test_reference::symmetric_eigen(
            N, symmetric.data(), eigenvalues.data()));
        REQUIRE(eigenvalues.minCoeff() >= -2.0e-9 * std::max<Float>(1.0, max_abs));
    }
}
}  // namespace

TEST_CASE("friction Hessian lift skips only a provably redundant projection",
          "[cuda][contact][friction][hessian]")
{
    SECTION("6D")
    {
        validate_lift<6>();
    }
    SECTION("9D")
    {
        validate_lift<9>();
    }
    SECTION("12D")
    {
        validate_lift<12>();
    }
}
