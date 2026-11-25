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
#include <line_search/cubic_line_minimizer.h>
#include <cmath>

using namespace uipc;
using namespace uipc::backend::cuda;
using Catch::Approx;

namespace
{
// phi(a) = E0 + slope * a + c2 * a^2 + c3 * a^3
struct Cubic
{
    Float E0, slope, c2, c3;
    Float operator()(Float a) const
    {
        return E0 + slope * a + c2 * a * a + c3 * a * a * a;
    }
    Float minimizer() const
    {
        if(c3 == 0.0)
            return -slope / (2.0 * c2);
        return (-c2 + std::sqrt(c2 * c2 - 3.0 * c3 * slope)) / (3.0 * c3);
    }
};

Float interpolate(const Cubic& phi, Float a0, Float a1)
{
    return cubic_line_minimizer(phi.E0, phi.slope, a0, phi(a0), a1, phi(a1));
}
}  // namespace

TEST_CASE("cubic line minimizer recovers the interpolated cubic", "[line search]")
{
    SECTION("positive quadratic and cubic coefficients")
    {
        Cubic phi{1.0, -0.25, 0.25, 1.0};
        CHECK(interpolate(phi, 0.5, 1.0) == Approx(phi.minimizer()).epsilon(1e-12));
        // the order of the two trials does not matter
        CHECK(interpolate(phi, 1.0, 0.5) == Approx(phi.minimizer()).epsilon(1e-12));
    }

    SECTION("negative quadratic coefficient")
    {
        Cubic phi{1.0, -0.3, -0.2, 1.0};
        CHECK(interpolate(phi, 0.25, 1.0) == Approx(phi.minimizer()).epsilon(1e-12));
    }

    SECTION("negative cubic coefficient")
    {
        Cubic phi{2.0, -1.0, 3.0, -0.5};
        CHECK(interpolate(phi, 0.25, 1.0) == Approx(phi.minimizer()).epsilon(1e-12));
    }

    SECTION("quadratic")
    {
        Cubic phi{0.5, -2.0, 4.0, 0.0};
        CHECK(interpolate(phi, 0.5, 1.0) == Approx(0.25).epsilon(1e-12));
    }

    SECTION("backtracked step at line-search energy scales")
    {
        // the full step raises the energy, the half step lowers it
        Cubic phi{1e-3, -4e-8, 2e-8, 5e-8};
        REQUIRE(phi(1.0) > phi.E0);
        REQUIRE(phi(0.5) <= phi.E0);
        Float a = interpolate(phi, 0.5, 1.0);
        CHECK(a == Approx(phi.minimizer()).epsilon(1e-9));
        CHECK(phi(a) <= phi(0.5));
    }
}

TEST_CASE("cubic line minimizer rejects cubics without a minimizer", "[line search]")
{
    // decreasing everywhere on a > 0
    Cubic decreasing{1.0, -1.0, -0.5, -0.25};
    CHECK(interpolate(decreasing, 0.5, 1.0) < 0.0);

    // ascent direction
    Cubic ascent{1.0, 1.0, 1.0, 0.0};
    CHECK(interpolate(ascent, 0.5, 1.0) < 0.0);

    // degenerate trials
    CHECK(cubic_line_minimizer(1.0, -1.0, 0.5, 0.9, 0.5, 0.9) < 0.0);
    CHECK(cubic_line_minimizer(1.0, -1.0, 0.0, 1.0, 0.5, 0.9) < 0.0);
}
