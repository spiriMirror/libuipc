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

#include <array>
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

TEST_CASE("ABD fused PCG update apply dot matches the block-Jacobi oracle",
          "[build_solve_focused][fused_pcg][abd_fused]")
{
    constexpr int BodyCount  = 10;
    constexpr int ScalarSize = BodyCount * 12;

    std::array<Matrix12x12, BodyCount> inv;
    for(int body = 0; body < BodyCount; ++body)
    {
        inv[body].setZero();
        for(int row = 0; row < 12; ++row)
        {
            inv[body](row, row) = 0.5 + 0.01 * body + 0.005 * row;
            if(row + 1 < 12)
            {
                inv[body](row, row + 1) = 0.002;
                inv[body](row + 1, row) = -0.001;
            }
        }
    }

    std::vector<Float> x(ScalarSize);
    std::vector<Float> p(ScalarSize);
    std::vector<Float> r(ScalarSize);
    std::vector<Float> Ap(ScalarSize);
    for(int i = 0; i < ScalarSize; ++i)
    {
        x[i]  = 0.01 * i;
        p[i]  = 0.02 * (i % 7 + 1);
        r[i]  = 0.4 + 0.003 * i;
        Ap[i] = 0.005 * (i % 5 + 1);
    }

    constexpr Float    Rz    = 4;
    constexpr Float    PAp   = 2;
    constexpr Float    Alpha = Rz / PAp;
    std::vector<Float> expected_x(ScalarSize);
    std::vector<Float> expected_r(ScalarSize);
    std::vector<Float> expected_z(ScalarSize);
    Float              expected_rz_new = 0;
    for(int body = 0; body < BodyCount; ++body)
    {
        Eigen::Matrix<Float, 12, 1> body_r;
        for(int row = 0; row < 12; ++row)
        {
            const int i   = body * 12 + row;
            expected_x[i] = x[i] + Alpha * p[i];
            expected_r[i] = r[i] - Alpha * Ap[i];
            body_r(row)   = expected_r[i];
        }
        const Eigen::Matrix<Float, 12, 1> body_z = inv[body] * body_r;
        for(int row = 0; row < 12; ++row)
        {
            const int i   = body * 12 + row;
            expected_z[i] = body_z(row);
            expected_rz_new += expected_r[i] * expected_z[i];
        }
    }

    cuda_tool::DeviceBuffer<Matrix12x12> d_inv(BodyCount);
    cuda_tool::DeviceDenseVector<Float>  d_x;
    cuda_tool::DeviceDenseVector<Float>  d_p;
    cuda_tool::DeviceDenseVector<Float>  d_r;
    cuda_tool::DeviceDenseVector<Float>  d_Ap;
    cuda_tool::DeviceDenseVector<Float>  d_z;
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
    cuda_tool::DeviceVar<Float>  d_rz_new{Float{0}};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    launch_fused_pcg_abd_update_apply_dot(d_inv.cview(),
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
    REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).margin(1e-9));

    SECTION("converged status is a full no-op")
    {
        d_x.buffer_view().fill(Float{7});
        d_r.buffer_view().fill(Float{11});
        d_z.buffer_view().fill(Float{13});
        d_rz_new    = Float{17};
        d_converged = IndexT{1};

        launch_fused_pcg_abd_update_apply_dot(d_inv.cview(),
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
