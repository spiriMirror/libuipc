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
                  Float                     tolerance = 1e-12)
{
    REQUIRE(actual.size() == expected.size());
    for(SizeT i = 0; i < actual.size(); ++i)
        REQUIRE(actual[i] == Catch::Approx(expected[i]).margin(tolerance));
}
}  // namespace

TEST_CASE("Fused PCG direction state publication is race-free",
          "[build_solve_focused][fused_pcg][direction_fused]")
{
    // More CTAs than can be resident at once exercises late consumers of the
    // published beta/countdown as well as a non-multiple tail.
    constexpr int ScalarCount = 256 * 1024 + 13;

    std::vector<Float> p(ScalarCount);
    std::vector<Float> z(ScalarCount);
    std::vector<Float> expected(ScalarCount);
    for(int i = 0; i < ScalarCount; ++i)
    {
        p[i]        = 0.01 * (i % 17 + 1);
        z[i]        = 0.02 * (i % 23 + 1);
        expected[i] = z[i] + Float{0.5} * p[i];
    }

    cuda_tool::DeviceDenseVector<Float> d_p;
    cuda_tool::DeviceDenseVector<Float> d_z;
    d_p.resize(ScalarCount);
    d_z.resize(ScalarCount);
    d_p.buffer_view().copy_from(p.data());
    d_z.buffer_view().copy_from(z.data());

    cuda_tool::DeviceVar<Float>  d_rz{Float{4}};
    cuda_tool::DeviceVar<Float>  d_rz_accum{Float{2}};
    cuda_tool::DeviceVar<Float>  d_published{Float{-11}};
    cuda_tool::DeviceVar<Float>  d_next_accum{Float{29}};
    cuda_tool::DeviceVar<Float>  d_beta{Float{-13}};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};
    cuda_tool::DeviceVar<Float>  d_rz_tol{Float{0.1}};

    launch_fused_pcg_publish_update_direction(d_p.view(),
                                              d_z.cview(),
                                              d_rz.view(),
                                              d_rz_accum.view(),
                                              d_published.view(),
                                              d_next_accum.view(),
                                              d_beta.view(),
                                              d_converged.view(),
                                              d_rz_tol.view(),
                                              nullptr);

    std::vector<Float> actual(ScalarCount);
    d_p.buffer_view().copy_to(actual.data());
    require_near(actual, expected);
    REQUIRE(static_cast<Float>(d_rz) == Float{2});
    REQUIRE(static_cast<Float>(d_published) == Float{2});
    REQUIRE(static_cast<Float>(d_next_accum) == Float{0});
    REQUIRE(static_cast<Float>(d_beta) == Float{0.5});
    REQUIRE(static_cast<IndexT>(d_converged) == IndexT{0});

    SECTION("convergence publishes the residual and leaves direction unchanged")
    {
        d_p.buffer_view().copy_from(p.data());
        d_rz         = Float{4};
        d_rz_accum   = Float{1e-12};
        d_published  = Float{-11};
        d_next_accum = Float{29};
        d_beta       = Float{-13};
        d_converged  = IndexT{0};
        d_rz_tol     = Float{1e-8};

        launch_fused_pcg_publish_update_direction(d_p.view(),
                                                  d_z.cview(),
                                                  d_rz.view(),
                                                  d_rz_accum.view(),
                                                  d_published.view(),
                                                  d_next_accum.view(),
                                                  d_beta.view(),
                                                  d_converged.view(),
                                                  d_rz_tol.view(),
                                                  nullptr);

        d_p.buffer_view().copy_to(actual.data());
        require_near(actual, p);
        REQUIRE(static_cast<Float>(d_rz) == Float{4});
        REQUIRE(static_cast<Float>(d_published) == Float{1e-12});
        REQUIRE(static_cast<Float>(d_next_accum) == Float{29});
        REQUIRE(static_cast<Float>(d_beta) == Float{-13});
        REQUIRE(static_cast<IndexT>(d_converged) == IndexT{1});

        // Later nodes captured in the same replay block must be full no-ops.
        d_published  = Float{31};
        d_rz_accum   = Float{7};
        d_next_accum = Float{37};
        launch_fused_pcg_publish_update_direction(d_p.view(),
                                                  d_z.cview(),
                                                  d_rz.view(),
                                                  d_rz_accum.view(),
                                                  d_published.view(),
                                                  d_next_accum.view(),
                                                  d_beta.view(),
                                                  d_converged.view(),
                                                  d_rz_tol.view(),
                                                  nullptr);
        d_p.buffer_view().copy_to(actual.data());
        require_near(actual, p);
        REQUIRE(static_cast<Float>(d_rz) == Float{4});
        REQUIRE(static_cast<Float>(d_published) == Float{31});
        REQUIRE(static_cast<Float>(d_next_accum) == Float{37});
        REQUIRE(static_cast<Float>(d_beta) == Float{-13});
        REQUIRE(static_cast<IndexT>(d_converged) == IndexT{1});
    }
}
