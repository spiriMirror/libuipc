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

#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
void require_near(const std::vector<Float>& actual,
                  const std::vector<Float>& expected,
                  Float                     tolerance = 1e-10)
{
    REQUIRE(actual.size() == expected.size());
    for(SizeT i = 0; i < actual.size(); ++i)
        REQUIRE(actual[i] == Catch::Approx(expected[i]).margin(tolerance));
}
}  // namespace

TEST_CASE("FEM fused PCG update apply dot matches the block-Jacobi oracle",
          "[build_solve_focused][fused_pcg][fem_fused]")
{
    constexpr int VertexCount = 513;
    constexpr int ScalarSize  = VertexCount * 3;

    std::vector<Matrix3x3> inv(VertexCount);
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        inv[vertex] << 0.5 + 0.0001 * vertex, 0.002, -0.001, -0.003,
            0.7 + 0.0002 * vertex, 0.004, 0.005, -0.002, 0.9 + 0.0003 * vertex;
    }

    std::vector<Float> x(ScalarSize);
    std::vector<Float> p(ScalarSize);
    std::vector<Float> r(ScalarSize);
    std::vector<Float> Ap(ScalarSize);
    for(int i = 0; i < ScalarSize; ++i)
    {
        x[i]  = 0.001 * i;
        p[i]  = 0.02 * (i % 7 + 1);
        r[i]  = 0.4 + 0.0003 * i;
        Ap[i] = 0.005 * (i % 5 + 1);
    }

    constexpr Float    Rz           = 4;
    constexpr Float    PAp          = 2;
    constexpr Float    Alpha        = Rz / PAp;
    constexpr Float    InitialRzNew = 0.75;
    std::vector<Float> expected_x(ScalarSize);
    std::vector<Float> expected_r(ScalarSize);
    std::vector<Float> expected_z(ScalarSize);
    Float              expected_rz_new = InitialRzNew;
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        Vector3 vertex_r;
        for(int component = 0; component < 3; ++component)
        {
            const int i         = vertex * 3 + component;
            expected_x[i]       = x[i] + Alpha * p[i];
            expected_r[i]       = r[i] - Alpha * Ap[i];
            vertex_r(component) = expected_r[i];
        }
        const Vector3 vertex_z = inv[vertex] * vertex_r;
        for(int component = 0; component < 3; ++component)
            expected_z[vertex * 3 + component] = vertex_z(component);
        expected_rz_new += vertex_r.dot(vertex_z);
    }

    cuda_tool::DeviceBuffer<Matrix3x3>  d_inv(VertexCount);
    cuda_tool::DeviceDenseVector<Float> d_x;
    cuda_tool::DeviceDenseVector<Float> d_p;
    cuda_tool::DeviceDenseVector<Float> d_r;
    cuda_tool::DeviceDenseVector<Float> d_Ap;
    cuda_tool::DeviceDenseVector<Float> d_z;
    d_x.resize(ScalarSize);
    d_p.resize(ScalarSize);
    d_r.resize(ScalarSize);
    d_Ap.resize(ScalarSize);
    d_z.resize(ScalarSize);
    d_inv.view().copy_from(inv.data());
    d_x.buffer_view().copy_from(x.data());
    d_p.buffer_view().copy_from(p.data());
    d_r.buffer_view().copy_from(r.data());
    d_Ap.buffer_view().copy_from(Ap.data());
    d_z.buffer_view().fill(Float{-1});

    cuda_tool::DeviceVar<Float>  d_rz{Rz};
    cuda_tool::DeviceVar<Float>  d_pAp{PAp};
    cuda_tool::DeviceVar<Float>  d_rz_new{InitialRzNew};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    launch_fused_pcg_fem_update_apply_dot(d_inv.cview(),
                                          d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          nullptr);

    std::vector<Float> actual_x(ScalarSize);
    std::vector<Float> actual_r(ScalarSize);
    std::vector<Float> actual_z(ScalarSize);
    d_x.buffer_view().copy_to(actual_x.data());
    d_r.buffer_view().copy_to(actual_r.data());
    d_z.buffer_view().copy_to(actual_z.data());
    require_near(actual_x, expected_x);
    require_near(actual_r, expected_r);
    require_near(actual_z, expected_z);
    REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).margin(1e-8));

    SECTION("converged status is a full no-op")
    {
        d_x.buffer_view().fill(Float{7});
        d_r.buffer_view().fill(Float{11});
        d_z.buffer_view().fill(Float{13});
        d_rz_new    = Float{17};
        d_converged = IndexT{1};

        launch_fused_pcg_fem_update_apply_dot(d_inv.cview(),
                                              d_x.view(),
                                              d_p.cview(),
                                              d_r.view(),
                                              d_Ap.cview(),
                                              d_z.view(),
                                              d_rz.view(),
                                              d_pAp.view(),
                                              d_rz_new.view(),
                                              d_converged.view(),
                                              nullptr);

        d_x.buffer_view().copy_to(actual_x.data());
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        require_near(actual_x, std::vector<Float>(ScalarSize, Float{7}));
        require_near(actual_r, std::vector<Float>(ScalarSize, Float{11}));
        require_near(actual_z, std::vector<Float>(ScalarSize, Float{13}));
        REQUIRE(static_cast<Float>(d_rz_new) == Float{17});
    }
}
